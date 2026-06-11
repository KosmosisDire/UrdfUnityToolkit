// Vendored minimal subset from UnityTechToolkit (Toolkit.MaterialExtensions).
// Only the members used by the URDF importer are kept. Detects the active render
// pipeline at runtime via shader names, so there is no compile-time URP/HDRP dependency.
// Originally adapted from Unity's URDF-Importer (Apache License 2.0).

using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace UrdfToolkit.Vendor
{
    public static class MaterialExtensions
    {
        public enum RenderPipelineType
        {
            Standard,
            URP,
            HDRP,
        }

        private static string[] standardShaders = { "Standard" };
        private static string[] hdrpShaders = { "HDRP/Lit" };
        private static string[] urpShaders = { "Universal Render Pipeline/Lit" };

        public static Material CreateBasicMaterial()
        {
            try
            {
                string[] shadersToTry = standardShaders;
                if (GetRenderPipelineType() == RenderPipelineType.HDRP)
                {
                    shadersToTry = hdrpShaders;
                }
                else if (GetRenderPipelineType() == RenderPipelineType.URP)
                {
                    shadersToTry = urpShaders;
                }

                foreach (var shaderName in shadersToTry)
                {
                    Shader shader = Shader.Find(shaderName);
                    if (shader != null)
                    {
                        var material = new Material(shader);
                        material.SetFloat("_Metallic", 0.75f);
                        material.SetFloat("_Glossiness", 0.75f);
                        return material;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogAssertion(ex.ToString());
            }
            return null;
        }

        /// Checks the current render pipeline. Used for creating the proper default material.
        public static RenderPipelineType GetRenderPipelineType()
        {
            if (GraphicsSettings.defaultRenderPipeline != null)
            {
                if (GraphicsSettings.defaultRenderPipeline.GetType().ToString().Contains("HighDefinition"))
                {
                    return RenderPipelineType.HDRP;
                }
                else if (GraphicsSettings.defaultRenderPipeline.GetType().ToString().Contains("Universal"))
                {
                    return RenderPipelineType.URP;
                }
            }
            return RenderPipelineType.Standard;
        }

        public static void SetMaterialColor(Material material, Color color)
        {
            material.SetColor(GetRenderPipelineType() != RenderPipelineType.Standard ? "_BaseColor" : "_Color", color);
        }

        public static void SetMaterialEmissionColor(Material material, Color color)
        {
            material.SetColor("_EmissionColor", color);
            material.EnableKeyword("_EMISSION");
        }
    }
}
