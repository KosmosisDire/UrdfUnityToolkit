using System;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UrdfToolkit.Urdf.Importer
{
    /// <summary>
    /// Builds Unity materials from URDF &lt;material&gt; definitions. Targets the Universal Render
    /// Pipeline (URP) exclusively: every material uses the "Universal Render Pipeline/Lit" shader and
    /// URP's property names (_BaseColor / _BaseMap).
    ///
    /// During an editor import the materials are written as .mat assets into a "materials" folder next
    /// to the source .urdf, so they can be reused and tweaked. At runtime, or when the source file
    /// lives outside the Unity project, plain in-memory materials are created instead.
    /// </summary>
    public static class UrdfMaterialImporter
    {
        private const string UrpLitShaderName = "Universal Render Pipeline/Lit";

        /// <summary>
        /// Builds (or, in the editor, loads/creates the .mat asset for) the material on a visual.
        /// Returns null when there is no material to build, leaving the geometry's existing materials
        /// untouched. <paramref name="fallbackName"/> names anonymous inline materials.
        /// </summary>
        public static Material Build(UrdfDescription urdf, UrdfMaterialDef? visualMaterial, string fallbackName)
        {
            if (visualMaterial == null) return null;

            var material = urdf.ResolveMaterial(visualMaterial.Value);

            // A reference to a name we never saw, or a material with neither color nor texture, has
            // nothing for us to build — leave the geometry's own materials in place.
            if (material.color == null && string.IsNullOrEmpty(material.texture)) return null;

            var name = !string.IsNullOrEmpty(material.name) ? material.name : fallbackName;
            var texturePath = string.IsNullOrEmpty(material.texture)
                ? null
                : urdf.ResolveTexturePath(material.texture);

#if UNITY_EDITOR
            if (TryToAssetPath(Path.GetDirectoryName(urdf.sourcePath), out var sourceDir))
                return BuildAsset(material, name, texturePath, sourceDir);
#endif
            return BuildRuntime(material, name, texturePath);
        }

        private static Material CreateUrpMaterial(UrdfMaterialDef def, string name)
        {
            var shader = Shader.Find(UrpLitShaderName);
            if (shader == null)
            {
                Debug.LogWarning($"[urdf] Shader '{UrpLitShaderName}' not found — is the Universal Render " +
                                 $"Pipeline installed? Falling back to the default shader for material '{name}'.");
                shader = Shader.Find("Standard");
            }

            var material = new Material(shader) { name = name };

            if (def.color != null)
            {
                var c = def.color.Value;
                var color = new Color(c.X, c.Y, c.Z, c.W);
                material.SetColor("_BaseColor", color);
                if (color.a < 1f) ConfigureTransparent(material);
            }

            return material;
        }

        // URP/Lit is opaque by default; reconfigure it for alpha blending so URDF colors with
        // alpha &lt; 1 actually render see-through.
        private static void ConfigureTransparent(Material material)
        {
            material.SetFloat("_Surface", 1f); // 0 = opaque, 1 = transparent
            material.SetFloat("_Blend", 0f);   // alpha blend
            material.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
            material.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
            material.SetFloat("_ZWrite", 0f);
            material.SetOverrideTag("RenderType", "Transparent");
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.DisableKeyword("_ALPHATEST_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.renderQueue = (int)RenderQueue.Transparent;
        }

#if UNITY_EDITOR
        private static Material BuildAsset(UrdfMaterialDef def, string name, string texturePath, string sourceDir)
        {
            var materialsDir = sourceDir + "/materials";
            if (!AssetDatabase.IsValidFolder(materialsDir))
                AssetDatabase.CreateFolder(sourceDir, "materials");

            var assetPath = $"{materialsDir}/{SanitizeFileName(name)}.mat";

            // Reuse an existing asset so visuals sharing a named material share one .mat, and
            // re-importing the robot doesn't pile up duplicates.
            var existing = AssetDatabase.LoadAssetAtPath<Material>(assetPath);
            if (existing != null) return existing;

            var material = CreateUrpMaterial(def, name);
            ApplyTextureAsset(material, texturePath);
            AssetDatabase.CreateAsset(material, assetPath);
            return material;
        }

        private static void ApplyTextureAsset(Material material, string texturePath)
        {
            if (string.IsNullOrEmpty(texturePath)) return;

            if (TryToAssetPath(texturePath, out var textureAssetPath))
            {
                var texture = AssetDatabase.LoadAssetAtPath<Texture2D>(textureAssetPath);
                if (texture != null)
                    material.SetTexture("_BaseMap", texture);
                else
                    Debug.LogWarning($"[urdf] could not load material texture asset at '{textureAssetPath}'.");
            }
            else
            {
                Debug.LogWarning($"[urdf] material texture '{texturePath}' lives outside the project; " +
                                 "it can't be referenced by a material asset and was skipped.");
            }
        }

        // Converts an absolute or already-relative path into a project-relative "Assets/..." path, but
        // only when it actually lives under this project's Assets folder.
        private static bool TryToAssetPath(string path, out string assetPath)
        {
            assetPath = null;
            if (string.IsNullOrEmpty(path)) return false;

            var normalized = path.Replace("\\", "/");
            if (normalized == "Assets" || normalized.StartsWith("Assets/"))
            {
                assetPath = normalized;
                return true;
            }

            var projectAssets = Application.dataPath.Replace("\\", "/"); // <project>/Assets
            var full = Path.GetFullPath(normalized).Replace("\\", "/");
            if (full == projectAssets || full.StartsWith(projectAssets + "/"))
            {
                assetPath = "Assets" + full.Substring(projectAssets.Length);
                return true;
            }

            return false;
        }

        private static string SanitizeFileName(string name)
        {
            foreach (var c in Path.GetInvalidFileNameChars())
                name = name.Replace(c, '_');
            return name;
        }
#endif

        private static Material BuildRuntime(UrdfMaterialDef def, string name, string texturePath)
        {
            var material = CreateUrpMaterial(def, name);

            if (!string.IsNullOrEmpty(texturePath) && File.Exists(texturePath))
            {
                var texture = LoadTextureFromFile(texturePath);
                if (texture != null) material.SetTexture("_BaseMap", texture);
            }

            return material;
        }

        private static Texture2D LoadTextureFromFile(string path)
        {
            try
            {
                var bytes = File.ReadAllBytes(path);
                if (Path.GetExtension(path).ToLowerInvariant() == ".tga")
                    return TGALoader.DecodeTGA(bytes);

                var texture = new Texture2D(2, 2);
                if (texture.LoadImage(bytes)) // handles png/jpg
                    return texture;

                Debug.LogWarning($"[urdf] unsupported or unreadable texture '{path}'.");
                return null;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[urdf] failed to load texture '{path}': {ex.Message}");
                return null;
            }
        }
    }
}
