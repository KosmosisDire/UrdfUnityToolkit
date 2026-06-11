using System.IO;
using System.Xml;

namespace UrdfToolkit.Xacro
{
    /// <summary>
    /// Converts a .xacro file into a plain .urdf by fully expanding xacro macros, properties,
    /// arguments, includes, conditionals and <c>${...}</c> expressions.
    /// </summary>
    public class XacroConverter
    {
        private readonly string inputFilePath;
        private readonly string outputFilePath;
        private readonly XmlDocument doc = new XmlDocument();

        public XacroConverter(string inputFilePath, string outputFilePath)
        {
            this.inputFilePath = inputFilePath;
            this.outputFilePath = outputFilePath;
        }

        public void Convert()
        {
            doc.Load(inputFilePath);

            // The package root is the folder containing the package (…/<pkg>/robots/foo.xacro -> …/<pkg>).
            string rootPath = Path.GetFullPath(Path.GetDirectoryName(Path.GetDirectoryName(inputFilePath)));

            using (var evaluator = new XacroExpressionEvaluator())
            {
                var processor = new XacroProcessor(doc, inputFilePath, rootPath, evaluator);
                processor.Process();
            }

            // Safety net: drop any xacro:* leftovers (e.g. comments-only namespaces) before saving.
            RemoveXacroElements(doc.DocumentElement);

            string outputDirectory = Path.GetDirectoryName(outputFilePath);
            if (!string.IsNullOrEmpty(outputDirectory) && !Directory.Exists(outputDirectory))
                Directory.CreateDirectory(outputDirectory);

            doc.Save(outputFilePath);
        }

        private static void RemoveXacroElements(XmlElement onNode)
        {
            XmlNodeList xacroNodes = onNode.SelectNodes("//*[starts-with(name(), 'xacro:')]");
            if (xacroNodes == null) return;

            foreach (XmlNode xacroNode in xacroNodes)
                xacroNode.ParentNode?.RemoveChild(xacroNode);
        }
    }
}
