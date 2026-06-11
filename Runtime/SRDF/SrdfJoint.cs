using System.Xml;
using UrdfToolkit.Extensions;
namespace UrdfToolkit.Srdf
{
    public class SrdfJoint
    {
        public string name;
        public double value;

        public SrdfJoint(XmlNode jointNode)
        {
            name = jointNode.GetString("name");
            value = jointNode.GetDouble("value");
        }
    }
}