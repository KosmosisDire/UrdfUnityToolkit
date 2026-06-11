using System.Collections.Generic;
using System.Linq;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Srdf
{

    public class SrdfGroup
    {
        private SrdfRobotDescription parent;
        public string name;
        private readonly List<string> joints = new();
        public List<string> JointTags => joints;
        public readonly List<string> links = new();
        public readonly List<string> subgroups = new();
        public readonly List<(string, string)> chains = new();

        private List<SrdfGroup> subGroupsCache;
        public List<SrdfGroup> SubGroups { get
        {
            if (subGroupsCache != null)
            {
                return subGroupsCache;
            }

            subGroupsCache = new List<SrdfGroup>();
            foreach (var group in parent.groups)
            {
                if (subgroups.Contains(group.name))
                {
                    subGroupsCache.Add(group);
                }
            }

            return subGroupsCache;
        }}

        private List<string> fullJointListCache;
        public List<string> FullJointList { get
        {
            if (fullJointListCache != null)
            {
                return fullJointListCache;
            }

            fullJointListCache = new List<string>(joints);
            foreach (var subgroup in SubGroups)
            {
                fullJointListCache.AddRange(subgroup.FullJointList);
            }

            // Remove duplicates
            fullJointListCache = fullJointListCache.Distinct().ToList();

            return fullJointListCache;
        }}

        public SrdfGroup(XmlNode groupNode, SrdfRobotDescription robotDescription)
        {
            parent = robotDescription;
            name = groupNode.GetString("name");

            var jointNodes = groupNode.SelectNodes("joint");
            foreach (XmlNode jointNode in jointNodes)
            {
                joints.Add(jointNode.GetString("name"));
            }

            var linkNodes = groupNode.SelectNodes("link");
            foreach (XmlNode linkNode in linkNodes)
            {
                links.Add(linkNode.GetString("name"));
            }

            var subgroupNodes = groupNode.SelectNodes("group");
            foreach (XmlNode subgroupNode in subgroupNodes)
            {
                subgroups.Add(subgroupNode.GetString("name"));
            }

            var chainNodes = groupNode.SelectNodes("chain");
            foreach (XmlNode chainNode in chainNodes)
            {
                chains.Add((chainNode.GetString("base_link"), chainNode.GetString("tip_link")));
            }
        }
    }

}