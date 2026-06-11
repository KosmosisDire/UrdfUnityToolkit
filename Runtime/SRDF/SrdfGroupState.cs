


using System.Collections.Generic;
using System.Xml;
using UrdfToolkit.Extensions;
namespace UrdfToolkit.Srdf
{
    public class SrdfGroupState
    {
        public string name;
        public string group;
        public List<SrdfJoint> joints = new List<SrdfJoint>();

        public SrdfGroupState(XmlNode groupStateNode)
        {
            name = groupStateNode.GetString("name");
            group = groupStateNode.GetString("group");
            
            var jointNodes = groupStateNode.SelectNodes("joint");
            foreach (XmlNode jointNode in jointNodes)
            {
                joints.Add(new SrdfJoint(jointNode));
            }
        }
    }
}