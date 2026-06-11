
using UnityEngine;
using System.Collections.Generic;
using System.IO;
using UrdfToolkit.Urdf;
using UrdfToolkit.Vendor;

namespace UrdfToolkit.Urdf.Importer
{
    public class UrdfCollisionMeshImporter : BaseMeshImporter
    {
        public static List<string> UsedTemplateFiles => s_UsedTemplateFiles;
        static List<string> s_UsedTemplateFiles = new List<string>();
        static List<string> s_CreatedAssetNames = new List<string>();
        
        public static void Create(Transform parent, UrdfGeometryDef geometry)
        {
            GameObject geometryGameObject = null;
            
            switch (geometry.type)
            {
                case GeometryTypes.Box:
                    geometryGameObject = new GameObject(geometry.type.ToString());
                    geometryGameObject.AddComponent<BoxCollider>();
                    break;
                case GeometryTypes.Cylinder:
                    geometryGameObject = CreateCylinderCollider();
                    break;
                case GeometryTypes.Sphere:
                    geometryGameObject = new GameObject(geometry.type.ToString());
                    geometryGameObject.AddComponent<SphereCollider>();
                    break;
                case GeometryTypes.Mesh:
                    geometryGameObject = CreateMeshCollider(geometry.mesh, parent);
                    break;
            }

            if (geometryGameObject != null)
            {
                geometryGameObject.transform.SetParent(parent, false);
                SetScale(parent, geometry);
            }
        }

        private static GameObject CreateMeshCollider(UrdfMesh mesh, Transform parent = null)
        {
            return CreateMeshColliderRuntime(mesh, parent);
        }

        private static GameObject CreateMeshColliderRuntime(UrdfMesh mesh, Transform parent = null)
        {
            GameObject meshObject = CreateMeshGameObjectRuntime(mesh, parent);
            if (meshObject != null)
            {
                ConvertMeshToColliders(meshObject);
            }
            return meshObject;
        }

        private static GameObject CreateCylinderCollider()
        {
            GameObject gameObject = new GameObject("Cylinder");
            MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();
            meshFilter.sharedMesh = CylinderMeshGenerator.Generate(2, 0.5f, 0.5f);
            ConvertCylinderToCollider(meshFilter);

            return gameObject;
        }

        private static void ConvertCylinderToCollider(MeshFilter filter)
        {
            GameObject go = filter.gameObject;
            var collider = filter.sharedMesh;
            MeshCollider current = go.AddComponent<MeshCollider>();
            current.sharedMesh = collider;
            current.convex = true;
            Object.DestroyImmediate(go.GetComponent<MeshRenderer>());
            Object.DestroyImmediate(filter);
        }

        private static void ConvertMeshToColliders(GameObject gameObject, string location = null, bool setConvex = true)
        {
            MeshFilter[] meshFilters = gameObject.GetComponentsInChildren<MeshFilter>();
            foreach (MeshFilter meshFilter in meshFilters)
            {
                GameObject child = meshFilter.gameObject;
                MeshCollider meshCollider = child.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = meshFilter.sharedMesh;

                meshCollider.convex = setConvex;

                Object.DestroyImmediate(child.GetComponent<MeshRenderer>());
                Object.DestroyImmediate(meshFilter);
            }
        }

        public static void BeginNewUrdfImport()
        {
            s_CreatedAssetNames.Clear();
            s_UsedTemplateFiles.Clear();
        }

    }
}