using System.Xml;
using UrdfToolkit.Extensions;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
    public struct UrdfAxisDef
    {
        public Vector3 xyz;

        /// <summary>
        /// Right, Up, Forward <br/>
        /// Used in the Unity coordinate system.
        /// </summary>
        public Vector3 xyzRUF;

        public UrdfAxisDef(XmlNode source)
        {
            xyz = source.GetVector3("xyz");
            xyzRUF = xyz.Ros2Unity();
        }

        public readonly string Stringify(int indentation)
        {
            return $"xyz: {xyz}".Indent(indentation);
        }
    }
}