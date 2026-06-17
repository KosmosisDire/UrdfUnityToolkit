using UnityEditor;
using UnityEngine;
using UrdfToolkit.Urdf;

namespace UrdfToolkit.Editor
{
    /// <summary>
    /// Replaces the default float-array inspector on <see cref="UrdfJointController"/> with one slider
    /// per controllable joint. Each slider is labelled with the joint name and ranges over the joint's
    /// URDF limits (degrees for revolute, metres for prismatic). Fixed joints have no DOF, so they are
    /// shown as a disabled row rather than a slider.
    /// </summary>
    [CustomEditor(typeof(UrdfJointController))]
    [CanEditMultipleObjects]
    public class UrdfJointControllerEditor : UnityEditor.Editor
    {
        // Fallback slider ranges for joints that declare no limits (e.g. continuous revolute joints).
        private const float DefaultAngleRange = 180f;  // degrees, ±
        private const float DefaultLinearRange = 1f;    // metres, ±

        public override void OnInspectorGUI()
        {
            var controller = (UrdfJointController)target;
            var robot = controller.GetComponent<UrdfRobot>();

            if (robot == null || robot.joints == null || robot.joints.Count == 0)
            {
                EditorGUILayout.HelpBox(
                    "No joints found. Import a robot, then make sure the UrdfRobot has its joints populated.",
                    MessageType.Info);
                return;
            }

            serializedObject.Update();

            // Keep the value array index-aligned with robot.joints so each slider drives the right joint.
            var jointsProp = serializedObject.FindProperty("joints");
            if (jointsProp.arraySize != robot.joints.Count)
                jointsProp.arraySize = robot.joints.Count;

            EditorGUILayout.LabelField("Joint Controls", EditorStyles.boldLabel);

            EditorGUI.BeginChangeCheck();

            for (int i = 0; i < robot.joints.Count; i++)
            {
                var joint = robot.joints[i];
                if (joint == null) continue;

                var label = joint.JointName;

                if (joint.IsFixed)
                {
                    using (new EditorGUI.DisabledScope(true))
                        EditorGUILayout.LabelField(label, "fixed");
                    continue;
                }

                GetSliderRange(joint, out float min, out float max);

                var element = jointsProp.GetArrayElementAtIndex(i);
                element.floatValue = EditorGUILayout.Slider(label, element.floatValue, min, max);
            }

            serializedObject.ApplyModifiedProperties();

            // Push the new targets to the pose immediately so the robot moves as the user drags, rather
            // than waiting for the next ExecuteAlways Update tick.
            if (EditorGUI.EndChangeCheck())
            {
                foreach (var t in targets)
                {
                    var c = (UrdfJointController)t;
                    var r = c.GetComponent<UrdfRobot>();
                    if (r != null) r.SetJointPositions(c.joints);
                }
                SceneView.RepaintAll();
            }
        }

        private static void GetSliderRange(UrdfJoint joint, out float min, out float max)
        {
            if (joint.HasLimits)
            {
                min = (float)joint.LowerLimit;
                max = (float)joint.UpperLimit;
                // Guard against an inverted or degenerate limit pair so the slider stays usable.
                if (max < min) (min, max) = (max, min);
                if (Mathf.Approximately(min, max)) max = min + Mathf.Epsilon;
                return;
            }

            // No declared limits (e.g. a continuous revolute joint): fall back to a symmetric range.
            float range = joint is UrdfJointPrismatic ? DefaultLinearRange : DefaultAngleRange;
            min = -range;
            max = range;
        }
    }
}
