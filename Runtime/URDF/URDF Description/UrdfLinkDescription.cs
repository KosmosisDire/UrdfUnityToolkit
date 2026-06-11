using System.Collections.Generic;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

public struct UrdfLinkDef
{
    public string name;
    public UrdfInertialDef? inertial;
    // A link may have any number of <visual> and <collision> elements (per the URDF spec); their
    // union forms the link's representation. We keep them all rather than just the first.
    public List<UrdfVisualDef> visuals;
    public List<UrdfCollisionDef> collisions;

    public UrdfLinkDef(XmlNode source)
    {
        name = source.Attributes?["name"]?.Value ?? "";
        var inertialNode = source.SelectSingleNode("inertial");

        inertial = null;
        if (inertialNode != null)
        {
            inertial = new UrdfInertialDef(inertialNode);
        }

        visuals = new List<UrdfVisualDef>();
        var visualNodes = source.SelectNodes("visual");
        if (visualNodes != null)
        {
            foreach (XmlNode visualNode in visualNodes)
            {
                visuals.Add(new UrdfVisualDef(visualNode));
            }
        }

        collisions = new List<UrdfCollisionDef>();
        var collisionNodes = source.SelectNodes("collision");
        if (collisionNodes != null)
        {
            foreach (XmlNode collisionNode in collisionNodes)
            {
                collisions.Add(new UrdfCollisionDef(collisionNode));
            }
        }
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"link: {name}";

        var strIn = "";
        if (inertial.HasValue) strIn += $"\ninertial:\n{inertial?.Stringify(indentation)}";
        foreach (var visual in visuals) strIn += $"\nvisual:\n{visual.Stringify(indentation)}";
        foreach (var collision in collisions) strIn += $"\ncollision:\n{collision.Stringify(indentation)}";
        strIn = strIn.Indent(indentation);

        return (str + strIn).Indent(indentation);
    }
}

}