
using System.Collections.Generic;
using System.IO;
using System.Xml;
using UnityEngine;
using UrdfToolkit;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{
public struct UrdfDescription
{
    public XmlDocument document;
    public string source;
    public string sourcePath;
    public string name;
    public Dictionary<string, UrdfLinkDef> links;
    public Dictionary<string, UrdfJointDef> joints;
    // Robot-level <material> definitions, keyed by name. Visuals can reference these by name.
    public Dictionary<string, UrdfMaterialDef> materials;
    public List<UrdfMesh> meshes;
    // Cached ROS package index, reused to resolve both mesh and texture file references.
    private RosPackageIndex packages;

    public UrdfDescription(string source, string sourcePath = null)
    {
        this.source = source;
        this.sourcePath = sourcePath;
        this.name = "";
        this.links = new Dictionary<string, UrdfLinkDef>();
        this.joints = new Dictionary<string, UrdfJointDef>();
        this.materials = new Dictionary<string, UrdfMaterialDef>();
        this.meshes = new List<UrdfMesh>();
        this.packages = null;
        document = new XmlDocument();

        Parse();

        // find likely paths for mesh files
        foreach (var link in links)
        {
            foreach (var visual in link.Value.visuals)
            {
                if (visual.geometry.mesh != null)
                {
                    meshes.Add(visual.geometry.mesh);
                }
            }

            foreach (var collision in link.Value.collisions)
            {
                if (collision.geometry.mesh != null)
                {
                    meshes.Add(collision.geometry.mesh);
                }
            }
        }

        packages = new RosPackageIndex(sourcePath);
        foreach (var mesh in meshes)
        {
            mesh.meshPath = ResolveAssetPath(mesh.filename, sourcePath, packages);
            if (mesh.meshPath == null)
                Debug.LogWarning($"[urdf] could not locate mesh '{mesh.filename}'");
        }
    }

    /// <summary>
    /// Resolves a mesh or texture reference to an absolute file path. <c>package://pkg/...</c> URLs
    /// go through the ROS package index; anything else (or a package that isn't indexed) falls back
    /// to walking up from the URDF file looking for the path. Returns null if nothing is found.
    /// </summary>
    private static string ResolveAssetPath(string filename, string sourcePath, RosPackageIndex packages)
    {
        if (string.IsNullOrEmpty(filename)) return null;

        var resolved = packages.ResolvePackageUrl(filename);
        if (!string.IsNullOrEmpty(resolved) && File.Exists(resolved))
            return resolved.Replace("\\", "/");

        var relative = filename.Replace("package://", "");
        var dir = Path.GetDirectoryName(sourcePath);
        for (int i = 0; i < 8 && !string.IsNullOrEmpty(dir); i++)
        {
            var candidate = Path.Combine(dir, relative);
            if (File.Exists(candidate))
                return candidate.Replace("\\", "/");
            dir = Path.GetDirectoryName(dir);
        }

        return null;
    }


    private void Parse()
    {
        document.LoadXml(source);
        XmlNode robotNode = document.SelectSingleNode("robot")!;
        if (robotNode == null) return;

        name = robotNode.GetString("name");

        var linkNodes = robotNode?.SelectNodes("link");
        var jointNodes = robotNode?.SelectNodes("joint");
        var materialNodes = robotNode?.SelectNodes("material");

        for (int i = 0; i < materialNodes?.Count; i++)
        {
            var materialNode = materialNodes[i];
            if (materialNode == null) continue;
            var material = new UrdfMaterialDef(materialNode);
            if (!string.IsNullOrEmpty(material.name))
                materials[material.name] = material; // indexer: last definition for a name wins
        }

        for (int i = 0; i < linkNodes?.Count; i++)
        {
            var linkNode = linkNodes[i];
            if (linkNode == null) continue;
            var link = new UrdfLinkDef(linkNode);
            links.Add(link.name ?? "", link);
        }

        for (int i = 0; i < jointNodes?.Count; i++)
        {
            var jointNode = jointNodes[i];
            if (jointNode == null) continue;
            var joint = new UrdfJointDef(jointNode);
            joints.Add(joint.name ?? "", joint);
        }
    }

    /// <summary>
    /// Resolves the material attached to a visual. A bare <c>&lt;material name="x"/&gt;</c> (no inline
    /// color or texture) is a reference to a robot-level material, so it is looked up in the table;
    /// an inline material is returned as-is.
    /// </summary>
    public readonly UrdfMaterialDef ResolveMaterial(UrdfMaterialDef visualMaterial)
    {
        var isReference = visualMaterial.color == null
                          && string.IsNullOrEmpty(visualMaterial.texture)
                          && !string.IsNullOrEmpty(visualMaterial.name);

        if (isReference && materials != null && materials.TryGetValue(visualMaterial.name, out var named))
            return named;

        return visualMaterial;
    }

    /// <summary>
    /// Resolves a material texture reference to an absolute file path, using the same lookup rules as
    /// mesh references. Returns null if the file cannot be located.
    /// </summary>
    public readonly string ResolveTexturePath(string filename)
    {
        return ResolveAssetPath(filename, sourcePath, packages);
    }

    public UrdfJointDef? GetJointForLink(string linkName)
    {
        foreach (var joint in joints)
        {
            if (joint.Value.child == linkName)
            {
                return joint.Value;
            }
        }
        
        return null;
    }

    public List<UrdfLinkDef> GetChildrenForLink(string linkName)
    {
        var children = new List<UrdfLinkDef>();
        foreach (var joint in joints)
        {
            if (joint.Value.parent == linkName)
            {
                if (links.TryGetValue(joint.Value.child, out var childLink))
                    children.Add(childLink);
                else
                    Debug.LogWarning($"[urdf] joint '{joint.Key}' references unknown child link '{joint.Value.child}'");
            }
        }
        return children;
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"robot: {name}";
        str += StringifyLinks(indentation);
        str += StringifyJoints(indentation);
        return str;
    }

    public readonly string StringifyLinks(int indentation)
    {
        var str = "";
        foreach (var link in links)
        {
            str += $"\n{link.Value.Stringify(indentation)}";
        }
        return str;
    }

    public readonly string StringifyJoints(int indentation)
    {
        var str = "";
        foreach (var joint in joints)
        {
            str += $"\n{joint.Value.Stringify(indentation)}";
        }
        return str;
    }

    public override string ToString()
    {
        return Stringify(2);
    }
}

}