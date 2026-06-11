using UrdfToolkit.Srdf;

namespace UrdfToolkit
{
    public static class SRDF
    {
        public static SrdfRobotDescription Parse(string srdfText)
        {
            return new SrdfRobotDescription(srdfText);
        }
    }
}