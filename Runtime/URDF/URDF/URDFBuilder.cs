using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using UrdfToolkit.Urdf.Importer;
using UrdfToolkit.Urdf;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
public class URDFBuilder : MonoBehaviour
{
    public static UrdfRobot Build(string urdfPath)
    {
        if (!File.Exists(urdfPath))
        {
            Debug.LogError($"URDF file not found at {urdfPath}");
            return null;
        }

        var urdfText = File.ReadAllText(urdfPath);

        return BuildRuntime(urdfText, null, urdfPath);
    }

    public static UrdfRobot BuildRuntime(string robotDescription, GameObject buildOnObject = null, string urdfPath = null)
    {
        var pathUndefined = string.IsNullOrEmpty(urdfPath);
        if (pathUndefined)
        {
            urdfPath = Path.Combine(Application.persistentDataPath, "temp_urdf_meshes");
            Debug.Log(urdfPath);
            Debug.Log(Application.persistentDataPath);
            if (!Directory.Exists(urdfPath))
            {
                Directory.CreateDirectory(urdfPath);
            }
        }

        var urdfDescription = new UrdfDescription(robotDescription, urdfPath);

        // download meshes if no path was given in which to find them
        if (pathUndefined)
        {
            for (var i = 0; i < urdfDescription.meshes.Count; i++)
            {
                var mesh = urdfDescription.meshes[i];
                var fileStat = new FileInfo(mesh.filename);
                if (!fileStat.Exists)
                {
                    Debug.LogError($"Mesh file {mesh.filename} not found on server");
                    continue;
                }

                // if the file is already downloaded and up to date, skip it
                var savePath = Path.Combine(urdfPath, mesh.filename.Replace("package://", ""));
                var fileInfo = new FileInfo(savePath);
                var localModTime = fileInfo.LastWriteTimeUtc.Subtract(DateTime.UnixEpoch).TotalMilliseconds;
                if (fileInfo.Exists && localModTime >= fileStat.LastWriteTimeUtc.Subtract(DateTime.UnixEpoch).TotalMilliseconds)
                {
                    Debug.Log($"Mesh file {mesh.filename} already up to date");
                    continue;
                }


                var fileData = File.ReadAllBytes(mesh.filename);
                if (fileData != null)
                {
                    // create directory if it doesn't exist
                    var directory = Path.GetDirectoryName(savePath);
                    if (!Directory.Exists(directory))
                    {
                        Directory.CreateDirectory(directory);
                    }

                    File.WriteAllBytes(savePath, fileData);
                }
            }
        }

        if (buildOnObject == null)
        {
            buildOnObject = new GameObject(urdfDescription.name);
        }
        var robot = buildOnObject.GetComponent<UrdfRobot>() ?? buildOnObject.AddComponent<UrdfRobot>();
        robot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

        var tree = BuildRecursive(FindRootLink(urdfDescription), urdfDescription, null, robot);
        tree.transform.SetParent(robot.transform);

        robot.PopulateJoints();
        robot.PopulateLinks();

        // make root articulation bodies immovable
        foreach (var body in robot.GetComponentsInChildren<ArticulationBody>())
        {
            if (body.transform.parent.GetComponent<ArticulationBody>() == null)
            {
                body.immovable = true;
            }
        }

        return robot;
    }

    /// <summary>
    /// The root link is the one that is not the child of any joint. (Using the first link in the
    /// dictionary is wrong — that's just whichever link appears first in the URDF.)
    /// </summary>
    private static UrdfLinkDef FindRootLink(UrdfDescription urdf)
    {
        var childLinks = new HashSet<string>();
        foreach (var joint in urdf.joints.Values)
        {
            if (!string.IsNullOrEmpty(joint.child))
                childLinks.Add(joint.child);
        }

        foreach (var link in urdf.links.Values)
        {
            if (!childLinks.Contains(link.name))
                return link;
        }

        // Degenerate description (cycle, or no links): fall back to the first one.
        return urdf.links.Values.First();
    }

    public static UrdfLink BuildRecursive(UrdfLinkDef linkData, UrdfDescription urdf, UrdfLink parent, UrdfRobot robot)
    {
        var link = BuildLink(linkData, urdf, parent);
        link.robot = robot;
        parent = link;

        var children = urdf.GetChildrenForLink(linkData.name);
        foreach (var child in children)
        {
            BuildRecursive(child, urdf, parent, robot);
        }

        return link;
    }

    private static UrdfLink BuildLink(UrdfLinkDef linkData, UrdfDescription? urdf, UrdfLink parent)
    {
        var link = new GameObject(linkData.name).AddComponent<UrdfLink>();
        link.name = linkData.name;

        if (linkData.inertial.HasValue)
        {
            link.mass = linkData.inertial.Value.mass;
            var inertialOrigin = linkData.inertial.Value.origin;
            if (inertialOrigin.HasValue)
            {
                link.hasCenterOfMass = true;
                link.centerOfMass = inertialOrigin.Value.xyzRUF;
            }
        }

        link.transform.SetParent(parent?.transform ?? null);

        // A link can carry several <visual>/<collision> elements; build one child per element.
        for (var i = 0; i < linkData.visuals.Count; i++)
        {
            var visual = linkData.visuals[i];
            var visuals = new GameObject("visual").transform;
            visuals.SetParent(link.transform);
            visuals.localPosition = visual.origin?.xyzRUF ?? Vector3.zero;
            visuals.localRotation = visual.origin?.rotationRUF ?? Quaternion.identity;
            visuals.localScale = visual.geometry.box?.size ?? Vector3.one;

            var geometry = UrdfVisualMeshImporter.Create(visuals, visual.geometry, out var hasEmbeddedMaterials);

            // A mesh that ships its own materials (an OBJ with an .mtl, or a Collada file) keeps them;
            // only fall back to the URDF <material> when the mesh brought none.
            if (urdf.HasValue && geometry != null && !hasEmbeddedMaterials)
            {
                var material = UrdfMaterialImporter.Build(urdf.Value, visual.material, $"{linkData.name}_{i}");
                if (material != null) ApplyMaterial(geometry, material);
            }
        }

        foreach (var collision in linkData.collisions)
        {
            var colliders = new GameObject("collision").transform;
            colliders.SetParent(link.transform);
            colliders.localPosition = collision.origin?.xyzRUF ?? Vector3.zero;
            colliders.localRotation = collision.origin?.rotationRUF ?? Quaternion.identity;
            colliders.localScale = collision.geometry.box?.size ?? Vector3.one;

            UrdfCollisionMeshImporter.Create(colliders, collision.geometry);
        }
        
        var jointData = urdf?.GetJointForLink(linkData.name);
        if (jointData.HasValue)
        {
            link.transform.SetLocalPositionAndRotation
            (
                jointData?.origin?.xyzRUF ?? Vector3.zero,
                jointData?.origin?.rotationRUF ?? Quaternion.identity
            );

            var joint = UrdfJoint.Create(link.gameObject, jointData.Value);
            link.joint = joint;
            joint.parentLink = parent;
            joint.childLink = link;
            
        }

        if (parent != null) parent.childLinks.Add(link);

        return link;
    }

    /// <summary>
    /// Assigns the URDF material to every renderer under the visual. Only called when the mesh file
    /// brought no authored materials of its own — a file's embedded materials take precedence. All
    /// submesh slots get the same material.
    /// </summary>
    private static void ApplyMaterial(GameObject geometry, Material material)
    {
        foreach (var renderer in geometry.GetComponentsInChildren<MeshRenderer>(true))
        {
            var count = Mathf.Max(1, renderer.sharedMaterials.Length);
            var mats = new Material[count];
            for (var m = 0; m < count; m++) mats[m] = material;
            renderer.sharedMaterials = mats;
        }
    }

}
}
