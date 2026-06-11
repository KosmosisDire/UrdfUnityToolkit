// Vendored from UnityTechToolkit (global-namespace ButtonAttribute).
// Marker attribute placed on methods to render an inspector button.
// The drawer lives in the Editor assembly (UrdfRobotEditor) scoped to UrdfRobot,
// so it does not collide with TechToolkit's project-wide button editor.

using System;

namespace UrdfToolkit.Vendor
{
    [AttributeUsage(AttributeTargets.Method, Inherited = true, AllowMultiple = false)]
    public sealed class ButtonAttribute : Attribute
    {
        public string Name { get; }

        public ButtonAttribute(string name = null)
        {
            Name = name;
        }
    }
}
