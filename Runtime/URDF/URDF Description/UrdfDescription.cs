
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
    public List<UrdfMesh> meshes;

    public UrdfDescription(string source, string sourcePath = null)
    {
        this.source = source;
        this.sourcePath = sourcePath;
        this.name = "";
        this.links = new Dictionary<string, UrdfLinkDef>();
        this.joints = new Dictionary<string, UrdfJointDef>();
        this.meshes = new List<UrdfMesh>();
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

        var packages = new RosPackageIndex(sourcePath);
        foreach (var mesh in meshes)
        {
            mesh.meshPath = ResolveMeshPath(mesh.filename, sourcePath, packages);
            if (mesh.meshPath == null)
                Debug.LogWarning($"[urdf] could not locate mesh '{mesh.filename}'");
        }
    }

    /// <summary>
    /// Resolves a mesh reference to an absolute file path. <c>package://pkg/...</c> URLs go through
    /// the ROS package index; anything else (or a package that isn't indexed) falls back to walking
    /// up from the URDF file looking for the path. Returns null if nothing is found.
    /// </summary>
    private static string ResolveMeshPath(string filename, string sourcePath, RosPackageIndex packages)
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