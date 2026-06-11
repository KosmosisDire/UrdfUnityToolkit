using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{
    
    public struct UrdfInertialDef
    {
        public float mass;
        public UrdfOriginDef? origin;
        public UrdfInertia? inertia;

        public UrdfInertialDef(XmlNode source)
        {
            mass = source.SelectSingleNode("mass").GetFloat("value");

            var originNode = source.SelectSingleNode("origin");
            var inertiaNode = source.SelectSingleNode("inertia");

            origin = null;
            if (originNode != null) 
            {
                origin = new UrdfOriginDef(originNode);
            }

            inertia = null;
            if (inertiaNode != null) 
            {
                inertia = new UrdfInertia(inertiaNode);
            }
        }

        public readonly string Stringify(int indentation)
        {
            var str = $"mass: {mass}";
            if (origin.HasValue) str += $"\n{origin?.Stringify(0)}";
            if (inertia.HasValue) str += $"\n{inertia?.Stringify(0)}";
            return str.Indent(indentation);
        }
    }

    public struct UrdfInertia
    {
        public float ixx;
        public float ixy;
        public float ixz;
        public float iyy;
        public float iyz;
        public float izz;

        public UrdfInertia(XmlNode source)
        {
            ixx = source.GetFloat("ixx");
            ixy = source.GetFloat("ixy");
            ixz = source.GetFloat("ixz");
            iyy = source.GetFloat("iyy");
            iyz = source.GetFloat("iyz");
            izz = source.GetFloat("izz");
        }

        public readonly string Stringify(int indentation)
        {
            return $"ixx: {ixx}  ixy: {ixy}  ixz: {ixz}  iyy: {iyy}  iyz: {iyz}  izz: {izz}".Indent(indentation);
        }
    }


}