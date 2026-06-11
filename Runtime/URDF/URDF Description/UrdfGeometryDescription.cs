using System.Xml;
using UrdfToolkit.Extensions;
using UnityEngine;

namespace UrdfToolkit.Urdf
{

public enum GeometryTypes
{
    Box,
    Cylinder,
    Sphere,
    Mesh,
    None
}


public struct UrdfGeometryDef
{
    public GeometryTypes type;
    public UrdfBox? box;
    public UrdfCylinder? cylinder;
    public UrdfSphere? sphere;
    public UrdfMesh mesh;
    public UrdfIGeometryType geometry;

    public static GeometryTypes GetGeometryType(string type)
    {
        switch (type)
        {
            case "box":
                return GeometryTypes.Box;
            case "cylinder":
                return GeometryTypes.Cylinder;
            case "sphere":
                return GeometryTypes.Sphere;
            case "mesh":
                return GeometryTypes.Mesh;
            default:
                return GeometryTypes.None;
        }
    }

    public UrdfGeometryDef(XmlNode source)
    {
        box = null;
        cylinder = null;
        sphere = null;
        mesh = null;
        geometry = null;

        // Use the first element child, skipping comments and whitespace. (The xacro converter
        // preserves source comments, so a <geometry> block can have a comment as its FirstChild.)
        XmlNode geometryNode = null;
        foreach (XmlNode child in source.ChildNodes)
        {
            if (child is XmlElement)
            {
                geometryNode = child;
                break;
            }
        }

        type = GetGeometryType(geometryNode?.Name ?? "");

        switch (type)
        {
            case GeometryTypes.Box:
                box = new UrdfBox(geometryNode);
                geometry = box.Value;
                break;
            case GeometryTypes.Cylinder:
                cylinder = new UrdfCylinder(geometryNode);
                geometry = cylinder.Value;
                break;
            case GeometryTypes.Sphere:
                sphere = new UrdfSphere(geometryNode);
                geometry = sphere.Value;
                break;
            case GeometryTypes.Mesh:
                mesh = new UrdfMesh(geometryNode);
                geometry = mesh;
                break;
            case GeometryTypes.None:
                // No recognized geometry — leave everything null so the importers create nothing.
                break;
        }
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"type: {type}";
        if (box.HasValue) str += $"\n{box?.Stringify(indentation)}";
        if (cylinder.HasValue) str += $"\n{cylinder?.Stringify(indentation)}";
        if (sphere.HasValue) str += $"\n{sphere?.Stringify(indentation)}";
        if (mesh != null) str += $"\n{mesh?.Stringify(indentation)}";
        return str.Indent(indentation);
    }
}

public interface UrdfIGeometryType
{
    public Vector3 UnityScale { get; }
}

public struct UrdfBox : UrdfIGeometryType
{
    public Vector3 size;
    public Vector3 UnityScale => size.Ros2UnityScale();

    public UrdfBox(XmlNode source)
    {
        size = source.GetVector3("size", Vector3.one);
    }

    public readonly string Stringify(int indentation)
    {
        return $"size: {size}".Indent(indentation);
    }
}

public struct UrdfCylinder : UrdfIGeometryType
{
    public float radius;
    public float length;

    // Unity's primitive (and generated collision) cylinder is 2 units tall along its Y axis with
    // radius 0.5. A URDF cylinder is radius + length about its local Z, which maps to Unity's Y, so
    // no rotation is needed — only scale: radial axes (X,Z) by radius/0.5 = 2·radius, length axis (Y)
    // by length/2. (The old `(radius, length, radius).Ros2UnityScale()` put length on X and ignored
    // the primitive's 0.5 radius / height 2, so cylinders came out sideways and mis-proportioned.)
    public Vector3 UnityScale => new Vector3(2f * radius, 0.5f * length, 2f * radius);

    public UrdfCylinder(XmlNode source)
    {
        radius = source.GetFloat("radius");
        length = source.GetFloat("length");
    }

    public readonly string Stringify(int indentation)
    {
        return $"radius: {radius}  length: {length}".Indent(indentation);
    }
}

public struct UrdfSphere : UrdfIGeometryType
{
    public float radius;

    // Unity's primitive sphere has radius 0.5, so scale by radius/0.5 = 2·radius to hit the URDF
    // radius. (The old `(radius,…).Ros2UnityScale()` scaled by raw radius, making spheres half size.)
    public Vector3 UnityScale => new Vector3(2f * radius, 2f * radius, 2f * radius);

    public UrdfSphere(XmlNode source)
    {
        radius = source.GetFloat("radius");
    }

    public readonly string Stringify(int indentation)
    {
        return $"radius: {radius}".Indent(indentation);
    }
}

public class UrdfMesh : UrdfIGeometryType
{
    public string filename;
    public Vector3 scale;
    public Vector3 UnityScale => scale.Ros2UnityScale();
    public string meshPath;

    public UrdfMesh(XmlNode source)
    {
        filename = source.GetString("filename");
        scale = source.GetVector3("scale", Vector3.one);
        meshPath = null;
    }

    public string Stringify(int indentation)
    {
        var str = $"filename: {filename}";
        str += $"\nlikelyPath: {meshPath}";
        return str.Indent(indentation);
    }
}

}