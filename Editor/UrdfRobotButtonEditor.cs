using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UrdfToolkit.Urdf;
using UrdfToolkit.Vendor;

namespace UrdfToolkit.Editor
{
    // Draws inspector buttons for methods on UrdfRobot marked with [Button].
    // Scoped to UrdfRobot (not all MonoBehaviours) so it doesn't collide with any
    // other project-wide button editor (e.g. TechToolkit's).
    [CustomEditor(typeof(UrdfRobot), true)]
    public class UrdfRobotButtonEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var methods = target.GetType()
                .GetMethods(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic)
                .Where(m => m.GetCustomAttributes(typeof(ButtonAttribute), true).Length > 0)
                .ToArray();

            if (methods.Length == 0)
            {
                return;
            }

            EditorGUILayout.Space();
            foreach (var method in methods)
            {
                var buttonAttribute = (ButtonAttribute)method.GetCustomAttribute(typeof(ButtonAttribute));
                string buttonName = string.IsNullOrEmpty(buttonAttribute.Name) ? method.Name : buttonAttribute.Name;

                if (GUILayout.Button(buttonName))
                {
                    foreach (var t in targets)
                    {
                        method.Invoke(t, null);
                    }
                }
            }
        }
    }
}
