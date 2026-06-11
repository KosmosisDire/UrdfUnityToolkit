using System.Collections.Generic;
using UrdfToolkit.Extensions;
using System.Xml;

namespace UrdfToolkit.Srdf
{
public class SrdfRobotDescription
{
    public XmlDocument document;
    public List<SrdfGroup> groups = new List<SrdfGroup>();
    public List<SrdfEndEffector> endEffectors = new List<SrdfEndEffector>();
    public List<SrdfGroupState> groupStates = new List<SrdfGroupState>();
    public List<SrdfDisableCollisions> disableCollisions = new List<SrdfDisableCollisions>();

    public string robotName;

    public SrdfRobotDescription(string source)
    {
        document = new XmlDocument();
        document.LoadXml(source);

        var robotNode = document.SelectSingleNode("robot");
        robotName = robotNode.GetString("name");

        var groupNodes = document.SelectNodes("robot/group");
        foreach (XmlNode groupNode in groupNodes)
        {
            groups.Add(new SrdfGroup(groupNode, this));
        }

        var endEffectorNodes = document.SelectNodes("robot/end_effector");
        foreach (XmlNode endEffectorNode in endEffectorNodes)
        {
            endEffectors.Add(new SrdfEndEffector(endEffectorNode));
        }

        var groupStateNodes = document.SelectNodes("robot/group_state");
        foreach (XmlNode groupStateNode in groupStateNodes)
        {
            groupStates.Add(new SrdfGroupState(groupStateNode));
        }

        var disableCollisionNodes = document.SelectNodes("robot/disable_collisions");
        foreach (XmlNode disableCollisionNode in disableCollisionNodes)
        {
            disableCollisions.Add(new SrdfDisableCollisions(disableCollisionNode));
        }

        var groupNames = string.Join(", ", groups.ConvertAll(g => g.name));
    }

    public SrdfEndEffector GetEndEffectorForGroup(string groupName)
    {
        return endEffectors.Find(ee => ee.group == groupName);
    }

    public string GetBaseJointForGroup(string groupName)
    {
        var group = groups.Find(g => g.name == groupName);
        return group?.FullJointList?[0];
    }

    public string GetTipJointForGroup(string groupName)
    {
        var group = groups.Find(g => g.name == groupName);
        var jointList = group?.FullJointList;
        return jointList?[^1];
    }
}
}