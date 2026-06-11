


using System.Xml;
using UrdfToolkit.Extensions;
namespace UrdfToolkit.Srdf
{
public class SrdfDisableCollisions
{
    public string link1;
    public string link2;
    public string reason;

    public SrdfDisableCollisions(XmlNode disableCollisionsNode)
    {
        link1 = disableCollisionsNode.GetString("link1");
        link2 = disableCollisionsNode.GetString("link2");
        reason = disableCollisionsNode.GetString("reason");
    }
}
}