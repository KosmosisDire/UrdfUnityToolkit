using System;
using System.IO;
using UrdfToolkit.Vendor;
using UnityEngine;
using UrdfToolkit.Urdf;

namespace UrdfToolkit.Urdf.Importer
{

public class BaseMeshImporter
{
    /// <summary>
    /// Loads a mesh file into a GameObject hierarchy, dispatching on the file extension
    /// (.stl, .dae, .obj). Returns null and logs an error if the mesh could not be located,
    /// the format is unsupported, or the loader threw. Shared by the visual and collision
    /// importers so both handle the same set of formats.
    /// </summary>
    public static GameObject CreateMeshGameObjectRuntime(UrdfMesh mesh, Transform parent = null)
    {
        if (string.IsNullOrEmpty(mesh.meshPath))
        {
            Debug.LogError($"Unable to load mesh '{mesh.filename}': it could not be located on disk.");
            return null;
        }

        GameObject meshObject = null;
        try
        {
            string meshPath = mesh.meshPath;
            string lower = meshPath.ToLower();
            if (lower.EndsWith(".stl"))
            {
                meshObject = CreateStlGameObjectRuntime(meshPath, parent);
            }
            else if (lower.EndsWith(".dae"))
            {
                // Collada is imported in its native (ROS, Z-up) frame, is later be converted to Unity's frame
                float globalScale = ColladaAssetPostProcessor.ReadGlobalScale(meshPath);
                meshObject = MeshImporter.Load(meshPath, globalScale, globalScale, globalScale);
            }
            else if (lower.EndsWith(".obj"))
            {
                meshObject = MeshImporter.Load(meshPath);
            }
            else
            {
                Debug.LogError($"Unsupported mesh format for '{mesh.filename}' (supported: .stl, .dae, .obj).");
            }
        }
        catch (Exception ex)
        {
            Debug.LogAssertion(ex);
        }

        if (meshObject == null)
        {
            Debug.LogError($"Unable to load mesh: {mesh.filename}");
        }

        return meshObject;
    }

    public static void SetScale(Transform transform, UrdfGeometryDef geometry)
    {
        var localScale = geometry.geometry.UnityScale;

        if (geometry.type == GeometryTypes.Mesh)
        {
            localScale = Vector3.Scale(transform.localScale, localScale);
        }

        transform.localScale = localScale;
    }

    public static GameObject CreateStlGameObjectRuntime(string stlFile, Transform parent = null)
    {
        Mesh[] meshes = StlImporter.ImportMesh(stlFile);
        if (meshes == null)
        {
            return null;
        }
        
        if (parent == null) parent = new GameObject(Path.GetFileNameWithoutExtension(stlFile)).transform;

        Material material = MaterialExtensions.CreateBasicMaterial();
        
        for (int i = 0; i < meshes.Length; i++)
        {
            GameObject gameObject = new GameObject(Path.GetFileNameWithoutExtension(stlFile));
            gameObject.AddComponent<MeshFilter>().sharedMesh = meshes[i];
            gameObject.AddComponent<MeshRenderer>().sharedMaterial = material;
            gameObject.transform.SetParent(parent, false);
        }
        return parent.gameObject;
    }

}

}