// Vendored minimal subset from UnityTechToolkit (Toolkit.UnityObjectExtentions).
// Only DestroyImmediateIfExists is used by the URDF importer.

using UnityEngine;

namespace UrdfToolkit.Vendor
{
    public static class UnityObjectExtensions
    {
        public static void DestroyImmediateIfExists<T>(this GameObject gameObject) where T : Component
        {
            var component = gameObject.GetComponent<T>();
            if (component != null)
            {
                GameObject.DestroyImmediate(component);
            }
        }
    }
}
