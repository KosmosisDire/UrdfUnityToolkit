using System.IO;
using UrdfToolkit.Urdf;

namespace UrdfToolkit
{

    public static class URDF
    {
        public static UrdfDescription Parse(string sourcePath)
        {
            var text = File.ReadAllText(sourcePath);
            var urdfDescription = new UrdfDescription(text, sourcePath);
            return urdfDescription;
        }

        public static UrdfDescription Parse(string text, string sourcePath)
        {
            var urdfDescription = new UrdfDescription(text, sourcePath);
            return urdfDescription;
        }
    }

}


