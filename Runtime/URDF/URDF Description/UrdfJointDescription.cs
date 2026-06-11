using System.Xml;
using UrdfToolkit.Extensions;


namespace UrdfToolkit.Urdf
{

public enum UrdfJointType
{
    Fixed,
    Continuous,
    Revolute,
    Prismatic,
    Floating,
    Planar
}

public struct UrdfJointDef
{
    public string name;
    public UrdfJointType type;
    public string parent;
    public string child;
    public UrdfOriginDef? origin;
    public UrdfAxisDef? axis;
    public UrdfLimitDef? limit;
    public UrdfDynamicsDef? dynamics;

    private static UrdfJointType GetUrdfJointType(string type)
    {
        switch (type)
        {
            case "revolute":
                return UrdfJointType.Revolute;
            case "prismatic":
                return UrdfJointType.Prismatic;
            case "continuous":
                return UrdfJointType.Continuous;
            case "fixed":
                return UrdfJointType.Fixed;
            case "floating":
                return UrdfJointType.Floating;
            case "planar":
                return UrdfJointType.Planar;
            default:
                return UrdfJointType.Fixed;
        }
    }

    public UrdfJointDef(XmlNode source)
    {
        limit = new UrdfLimitDef();

        name = source.GetString("name");
        type = GetUrdfJointType(source.GetString("type"));
        parent = source.SelectSingleNode("parent")?.GetString("link") ?? "";
        child = source.SelectSingleNode("child")?.GetString("link") ?? "";

        var originNode = source.SelectSingleNode("origin");
        var axisNode = source.SelectSingleNode("axis");
        var limitNode = source.SelectSingleNode("limit");
        var dynamicsNode = source.SelectSingleNode("dynamics");

        origin = null;
        if (originNode != null) 
        {
            origin = new UrdfOriginDef(originNode);
        }

        axis = null;
        if (axisNode != null)
        {
            axis = new UrdfAxisDef(axisNode);
        }

        limit = null;
        if (limitNode != null)
        {
            limit = new UrdfLimitDef(limitNode);
        }

        dynamics = null;
        if (dynamicsNode != null)
        {
            dynamics = new UrdfDynamicsDef(dynamicsNode);
        }
    }

    public readonly string Stringify(int indentation)
    {
        var str = $"joint: {name}";

        var strIn = $"\ntype: {type}";
        strIn += $"\nparent: {parent}";
        strIn += $"\nchild: {child}";
        if (origin.HasValue) strIn += $"\norigin:\n{origin?.Stringify(indentation)}";
        if (axis.HasValue) strIn += $"\naxis:\n{axis?.Stringify(indentation)}";
        if (limit.HasValue) strIn += $"\nlimit:\n{limit?.Stringify(indentation)}";
        if (dynamics.HasValue) strIn += $"\ndynamics:\n{dynamics?.Stringify(indentation)}";
        strIn = strIn.Indent(indentation);

        return (str + strIn).Indent(indentation);
    }
}


}