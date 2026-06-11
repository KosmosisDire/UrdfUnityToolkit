using UrdfToolkit.Extensions;
using System.Xml;
namespace UrdfToolkit.Urdf
{

public struct UrdfVisualDef
{
    public UrdfGeometryDef geometry;
    public UrdfOriginDef? origin;
    public UrdfMaterialDef? material;

    public UrdfVisualDef(XmlNode source)
    {
        var originNode = source.SelectSingleNode("origin");
        var geometryNode = source.SelectSingleNode("geometry");
        var materialNode = source.SelectSingleNode("material");

        origin = null;
        if (originNode != null) 
        {
            origin = new UrdfOriginDef(originNode);
        }

        if (geometryNode == null) throw new System.Exception("Visual description must have a geometry node");   
        geometry = new UrdfGeometryDef(geometryNode);

        material = null;
        if (materialNode != null) 
        {
            material = new UrdfMaterialDef(materialNode);
        }
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"geometry:\n{geometry.Stringify(indentation)}";
        if (material.HasValue) str += $"\n{material?.Stringify(indentation)}";
        if (origin.HasValue) str += $"\n{origin?.Stringify(indentation)}";
        return str.Indent(indentation);
    }
}

}