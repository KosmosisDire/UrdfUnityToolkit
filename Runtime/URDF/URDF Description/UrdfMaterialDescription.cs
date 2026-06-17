using System.Numerics;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

    public struct UrdfMaterialDef
    {
        public string name;
        public Vector4? color;
        // Texture filename exactly as written in the URDF (e.g. "package://pkg/textures/foo.png").
        // Resolved to a real path when the material is built.
        public string texture;

        public UrdfMaterialDef(XmlNode source)
        {
            name = source.Attributes?["name"]?.Value ?? "";

            color = null;
            var colorNode = source.SelectSingleNode("color");
            if (colorNode != null)
            {
                var values = colorNode.Attributes?["rgba"]?.Value.Split(' ');
                if (values == null || values.Length != 4)
                {
                    color = new Vector4(0, 0, 0, 1);
                }
                else
                {
                    // Default alpha to 1 (opaque) when malformed — a 0 alpha would render invisible.
                    color = new Vector4(
                        values[0].ParseFloatOrDefault(0),
                        values[1].ParseFloatOrDefault(0),
                        values[2].ParseFloatOrDefault(0),
                        values[3].ParseFloatOrDefault(1));
                }
            }

            texture = source.SelectSingleNode("texture")?.Attributes?["filename"]?.Value;
        }

        public readonly string Stringify(int indentation)
        {
            var str = $"material: {name}";
            if (color.HasValue) str += $"  color: {color}";
            if (!string.IsNullOrEmpty(texture)) str += $"  texture: {texture}";
            return str.Indent(indentation);
        }
    }

}