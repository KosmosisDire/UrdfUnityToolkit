using System.Numerics;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

    public struct UrdfMaterialDef
    {
        public string name;
        public Vector4? color;

        public UrdfMaterialDef(XmlNode source)
        {
            name = source.Attributes?["name"]?.Value ?? "";
            
            var colorNode = source.SelectSingleNode("color");
            color = null;
            if (colorNode != null)
            {
                var values = colorNode.Attributes?["rgba"]?.Value.Split(' ');
                if (values?.Length != 4)
                {
                    color = new Vector4(0, 0, 0, 1);
                    return;
                }

                color = new Vector4(values[0].ParseFloatOrDefault(0), values[1].ParseFloatOrDefault(0), values[2].ParseFloatOrDefault(0), values[3].ParseFloatOrDefault(0));
            }

        }

        public readonly string Stringify(int indentation)
        {
            return $"color: {color}".Indent(indentation);
        }
    }

}