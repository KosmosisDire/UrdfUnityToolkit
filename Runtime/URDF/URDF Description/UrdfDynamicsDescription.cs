using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

public struct UrdfDynamicsDef
{
    public float damping;
    public float friction;

    public UrdfDynamicsDef(XmlNode source)
    {
        damping = source.GetFloat("damping");
        friction = source.GetFloat("friction");
    }

    public readonly string Stringify(int indentation)
    {
        return $"damping: {damping}\nfriction: {friction}".Indent(indentation);
    }
}

}