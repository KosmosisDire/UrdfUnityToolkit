using System.IO;
using UnityEditor;
using UnityEngine;
using UrdfToolkit.Urdf;
using UrdfToolkit.Xacro;

namespace UrdfToolkit.Editor
{
[DefaultExecutionOrder(-100000)]
public class URDFImporterExtension
{
    [MenuItem("Assets/Import Robot from this .urdf", true)]
    public static bool ImportURDFValid()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        return Path.GetExtension(assetPath)?.ToLower() == ".urdf";
    }

    [MenuItem("Assets/Import Robot from this .urdf")]
    public static void ImportURDF()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        if (Path.GetExtension(assetPath)?.ToLower() == ".urdf")
        {
            if (assetPath != "")
            {
                URDFBuilder.Build(assetPath);
                // Persist and surface any material assets written next to the .urdf.
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();
            }
        }
        else
        {
            EditorUtility.DisplayDialog("URDF Import", "The file you selected was not a URDF file. Please select a valid URDF file.", "Ok");
        }
    }

    [MenuItem("Assets/Import Robot from this .xacro", true)]
    public static bool ImportXACROValid()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        return Path.GetExtension(assetPath)?.ToLower() == ".xacro";
    }

    [MenuItem("Assets/Import Robot from this .xacro")]
    public static void ImportXACRO()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        var outputDir = Path.GetDirectoryName(Path.GetDirectoryName(assetPath));
        var outputPath = Path.Combine(outputDir, Path.GetFileNameWithoutExtension(assetPath) + ".urdf");
        new XacroConverter(assetPath, outputPath).Convert();
        URDFBuilder.Build(outputPath);

        // Persist material assets written next to the generated .urdf before removing it.
        AssetDatabase.SaveAssets();
        File.Delete(outputPath);
        AssetDatabase.Refresh();
    }

    [MenuItem("Assets/Convert this .xacro to .urdf", true)]
    public static bool ConvertXACROValid()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        return Path.GetExtension(assetPath)?.ToLower() == ".xacro";
    }

    [MenuItem("Assets/Convert this .xacro to .urdf")]
    public static void ConvertXACRO()
    {
        string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

        if (Path.GetExtension(assetPath)?.ToLower() != ".xacro")
        {
            EditorUtility.DisplayDialog("Xacro Convert", "The file you selected was not a .xacro file. Please select a valid .xacro file.", "Ok");
            return;
        }

        var outputDir = Path.GetDirectoryName(assetPath);
        var outputPath = Path.Combine(outputDir, Path.GetFileNameWithoutExtension(assetPath) + ".urdf");
        new XacroConverter(assetPath, outputPath).Convert();

        AssetDatabase.Refresh();
    }
}
}
