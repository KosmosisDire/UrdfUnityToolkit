using UrdfToolkit.Urdf;
using UnityEngine;

namespace UrdfToolkit.Urdf.Importer
{

public class UrdfVisualMeshImporter : BaseMeshImporter
{
    public static GameObject Create(Transform parent, UrdfGeometryDef geometry, out bool hasEmbeddedMaterials)
    {
        hasEmbeddedMaterials = false;
        GameObject geometryGameObject = null;

        switch (geometry.type)
        {
            case GeometryTypes.Box:
                geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
                var boxCollider = geometryGameObject.GetComponent<BoxCollider>();
                if (boxCollider) GameObject.DestroyImmediate(boxCollider);
                break;
            case GeometryTypes.Cylinder:
                geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                var cylinderCollider = geometryGameObject.GetComponent<CapsuleCollider>();
                if (cylinderCollider) GameObject.DestroyImmediate(cylinderCollider);
                break;
            case GeometryTypes.Sphere:
                geometryGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                var sphereCollider = geometryGameObject.GetComponent<SphereCollider>();
                if (sphereCollider) GameObject.DestroyImmediate(sphereCollider);
                break;
            case GeometryTypes.Mesh:
                geometryGameObject = CreateMeshVisual(geometry.mesh, out hasEmbeddedMaterials, parent);
                break;
        }

        if (geometryGameObject != null)
        {
            geometryGameObject.transform.SetParent(parent, false);
            geometryGameObject.transform.localScale = Vector3.one;
            SetScale(parent, geometry);
        }

        return geometryGameObject;
    }

    private static GameObject CreateMeshVisual(UrdfMesh mesh, out bool hasEmbeddedMaterials, Transform parent = null)
    {
        return CreateMeshVisualRuntime(mesh, out hasEmbeddedMaterials, parent);
    }

    private static GameObject CreateMeshVisualRuntime(UrdfMesh mesh, out bool hasEmbeddedMaterials, Transform parent = null)
    {
        return CreateMeshGameObjectRuntime(mesh, out hasEmbeddedMaterials, parent);
    }
}

}