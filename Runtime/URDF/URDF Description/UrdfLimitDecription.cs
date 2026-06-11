using System;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

public struct UrdfLimitDef
{
    public float lower;
    public float upper;
    public float effort;
    public float velocity;

    public UrdfLimitDef(XmlNode source)
    {
        // Store the raw URDF values. lower/upper are radians for revolute joints and metres for
        // prismatic joints, so the unit conversion is joint-type-specific and happens in each
        // joint's ImportJointData — not here, where the joint type is unknown.
        lower = source.Attributes?["lower"]?.Value.ParseFloatOrDefault(0) ?? 0;
        upper = source.Attributes?["upper"]?.Value.ParseFloatOrDefault(0) ?? 0;
        effort = source.Attributes?["effort"]?.Value.ParseFloatOrDefault(0) ?? 0;
        velocity = source.Attributes?["velocity"]?.Value.ParseFloatOrDefault(0) ?? 0;
    }

    public readonly string Stringify(int indentation)
    {
        return $"lower: {lower}  upper: {upper}  effort: {effort}  velocity: {velocity}".Indent(indentation);
    }
}

}