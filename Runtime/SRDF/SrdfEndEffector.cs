


using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Srdf
{
public class SrdfEndEffector
{
    public string name;
    public string parentLink;
    public string group;
    public string parentGroup;

    public SrdfEndEffector(XmlNode endEffectorNode)
    {
        name = endEffectorNode.GetString("name");
        parentLink = endEffectorNode.GetString("parent_link");
        group = endEffectorNode.GetString("group");
        parentGroup = endEffectorNode.GetString("parent_group");
    }

}
}