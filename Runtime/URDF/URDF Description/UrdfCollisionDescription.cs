using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

public struct UrdfCollisionDef
{
    public UrdfGeometryDef geometry;
    public UrdfOriginDef? origin;

    public UrdfCollisionDef(XmlNode source)
    {
        var originNode = source.SelectSingleNode("origin");
        var geometryNode = source.SelectSingleNode("geometry");

        origin = null;
        if (originNode != null) 
        {
            origin = new UrdfOriginDef(originNode);
        }

        if (geometryNode == null) throw new System.Exception("Collision description must have a geometry node");
        geometry = new UrdfGeometryDef(geometryNode);
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"geometry:\n{geometry.Stringify(indentation)}";
        if (origin.HasValue) str += $"\n{origin?.Stringify(indentation)}";
        return str.Indent(indentation);
    }
}

}
