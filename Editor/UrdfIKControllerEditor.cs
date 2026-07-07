using System;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UrdfToolkit.Urdf;

namespace UrdfToolkit.Editor
{
    /// <summary>
    /// Inspector and scene editing for <see cref="UrdfIKController"/>. Each constraint picks its link
    /// from a flat dropdown, its active axes and frame, and a weight, and is posed with handles in the
    /// scene while the robot is selected.
    /// </summary>
    [CustomEditor(typeof(UrdfIKController))]
    public class UrdfIKControllerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            var controller = (UrdfIKController)target;
            var robot = controller.GetComponent<UrdfRobot>();

            serializedObject.Update();

            var list = serializedObject.FindProperty("constraints");
            EditorGUILayout.LabelField("Constraints", EditorStyles.boldLabel);

            int removeAt = -1;
            for (int i = 0; i < list.arraySize; i++)
            {
                var element = list.GetArrayElementAtIndex(i);
                using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
                {
                    using (new EditorGUILayout.HorizontalScope())
                    {
                        EditorGUILayout.LabelField($"Constraint {i}", EditorStyles.boldLabel);
                        if (GUILayout.Button("Remove", GUILayout.Width(70))) removeAt = i;
                    }

                    var typeProp = element.FindPropertyRelative("type");
                    EditorGUILayout.PropertyField(typeProp, new GUIContent("Type"));

                    if (typeProp.enumValueIndex == (int)UrdfIKController.ConstraintType.Joint)
                    {
                        DrawJointConstraint(element, robot);
                    }
                    else
                    {
                        EditorGUILayout.PropertyField(element.FindPropertyRelative("target"), new GUIContent("Target"));

                        // No target Transform: the pose is stored on the constraint and editable here.
                        if (element.FindPropertyRelative("target").objectReferenceValue == null)
                            DrawStoredPose(element);

                        DrawLinkDropdown(element, robot);
                        EditorGUILayout.PropertyField(element.FindPropertyRelative("frame"), new GUIContent("Frame"));
                        DrawAxisRow("Position", element, "posX", "posY", "posZ");
                        DrawAxisRow("Rotation", element, "rotX", "rotY", "rotZ");
                        EditorGUILayout.PropertyField(element.FindPropertyRelative("weight"), new GUIContent("Weight"));
                    }
                }
            }

            if (GUILayout.Button("Add Constraint")) AddConstraint(list);
            if (removeAt >= 0) list.DeleteArrayElementAtIndex(removeAt);

            serializedObject.ApplyModifiedProperties();
        }

        // New constraints start clean: cartesian, position XYZ, local frame, weight 1.
        private static void AddConstraint(SerializedProperty list)
        {
            int i = list.arraySize;
            list.arraySize++;
            var e = list.GetArrayElementAtIndex(i);
            e.FindPropertyRelative("type").enumValueIndex = 0; // Cartesian
            e.FindPropertyRelative("jointName").stringValue = "";
            e.FindPropertyRelative("jointTarget").floatValue = 0f;
            e.FindPropertyRelative("target").objectReferenceValue = null;
            e.FindPropertyRelative("link").stringValue = "";
            e.FindPropertyRelative("position").vector3Value = Vector3.zero;
            e.FindPropertyRelative("rotation").quaternionValue = Quaternion.identity;
            e.FindPropertyRelative("frame").enumValueIndex = 1; // Local (enum is World=0, Local=1)
            e.FindPropertyRelative("weight").floatValue = 1f;
            e.FindPropertyRelative("posX").boolValue = true;
            e.FindPropertyRelative("posY").boolValue = true;
            e.FindPropertyRelative("posZ").boolValue = true;
            e.FindPropertyRelative("rotX").boolValue = false;
            e.FindPropertyRelative("rotY").boolValue = false;
            e.FindPropertyRelative("rotZ").boolValue = false;
        }

        // Editable stored pose, shown when a constraint has no target Transform.
        private static void DrawStoredPose(SerializedProperty element)
        {
            EditorGUILayout.PropertyField(element.FindPropertyRelative("position"), new GUIContent("Position"));

            var rotProp = element.FindPropertyRelative("rotation");
            Quaternion rot = rotProp.quaternionValue;
            if (rot.x == 0 && rot.y == 0 && rot.z == 0 && rot.w == 0) rot = Quaternion.identity;

            EditorGUI.BeginChangeCheck();
            Vector3 euler = EditorGUILayout.Vector3Field("Rotation", rot.eulerAngles);
            if (EditorGUI.EndChangeCheck()) rotProp.quaternionValue = Quaternion.Euler(euler);
        }

        // Joint-space constraint: pick a controllable joint and a target value (sliderable to limits).
        private static void DrawJointConstraint(SerializedProperty element, UrdfRobot robot)
        {
            var jointProp = element.FindPropertyRelative("jointName");
            var movable = robot != null && robot.joints != null
                ? robot.joints.Where(j => j != null && !j.IsFixed).ToArray()
                : Array.Empty<UrdfJoint>();

            if (movable.Length == 0)
            {
                EditorGUILayout.PropertyField(jointProp, new GUIContent("Joint"));
            }
            else
            {
                var names = movable.Select(j => j.JointName).ToArray();
                int current = Array.IndexOf(names, jointProp.stringValue);
                string[] options = names;
                int popup = current;
                if (current < 0)
                {
                    string missing = string.IsNullOrEmpty(jointProp.stringValue) ? "(none)" : jointProp.stringValue + " (missing)";
                    options = new[] { missing }.Concat(names).ToArray();
                    popup = 0;
                }

                EditorGUI.BeginChangeCheck();
                int selected = EditorGUILayout.Popup(new GUIContent("Joint"), popup, options.Select(n => new GUIContent(n)).ToArray());
                if (EditorGUI.EndChangeCheck())
                {
                    string chosen = current < 0 ? (selected > 0 ? names[selected - 1] : null) : names[selected];
                    if (chosen != null)
                    {
                        jointProp.stringValue = chosen;
                        var j = movable.FirstOrDefault(m => m.JointName == chosen);
                        if (j != null) element.FindPropertyRelative("jointTarget").floatValue = j.GetPosition();
                    }
                }
            }

            var targetProp = element.FindPropertyRelative("jointTarget");
            var joint = movable.FirstOrDefault(j => j.JointName == jointProp.stringValue);
            if (joint != null && joint.HasLimits)
                targetProp.floatValue = EditorGUILayout.Slider("Target", targetProp.floatValue, (float)joint.LowerLimit, (float)joint.UpperLimit);
            else
                EditorGUILayout.PropertyField(targetProp, new GUIContent("Target"));

            EditorGUILayout.PropertyField(element.FindPropertyRelative("weight"), new GUIContent("Weight"));
        }

        // Row of X/Y/Z toggle buttons for an axis mask.
        private static void DrawAxisRow(string label, SerializedProperty element, string x, string y, string z)
        {
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.PrefixLabel(label);
                ToggleButton(element.FindPropertyRelative(x), "X");
                ToggleButton(element.FindPropertyRelative(y), "Y");
                ToggleButton(element.FindPropertyRelative(z), "Z");
                GUILayout.FlexibleSpace();
            }
        }

        private static void ToggleButton(SerializedProperty boolProp, string text)
        {
            boolProp.boolValue = GUILayout.Toggle(boolProp.boolValue, text, "Button", GUILayout.Width(28));
        }

        // Flat popup of the robot's link names; keeps a stored-but-missing value visible.
        private static void DrawLinkDropdown(SerializedProperty element, UrdfRobot robot)
        {
            var linkProp = element.FindPropertyRelative("link");
            var label = new GUIContent("Link");
            var names = robot != null && robot.links != null
                ? robot.links.Where(l => l != null).Select(l => l.name).ToArray()
                : Array.Empty<string>();

            if (names.Length == 0) { EditorGUILayout.PropertyField(linkProp, label); return; }

            int current = Array.IndexOf(names, linkProp.stringValue);
            string[] options = names;
            int popup = current;
            if (current < 0)
            {
                string missing = string.IsNullOrEmpty(linkProp.stringValue) ? "(none)" : linkProp.stringValue + " (missing)";
                options = new[] { missing }.Concat(names).ToArray();
                popup = 0;
            }

            EditorGUI.BeginChangeCheck();
            int selected = EditorGUILayout.Popup(label, popup, options.Select(n => new GUIContent(n)).ToArray());
            if (EditorGUI.EndChangeCheck())
            {
                string chosen = current < 0 ? (selected > 0 ? names[selected - 1] : null) : names[selected];
                if (chosen != null)
                {
                    linkProp.stringValue = chosen;
                    SeedPoseFromLink(element, robot, chosen);
                }
            }
        }

        // Start a target-less constraint's stored pose at the chosen link's current transform.
        private static void SeedPoseFromLink(SerializedProperty element, UrdfRobot robot, string linkName)
        {
            if (element.FindPropertyRelative("target").objectReferenceValue != null) return;
            var link = robot != null ? robot.GetLink(linkName) : null;
            if (link == null) return;
            element.FindPropertyRelative("position").vector3Value = link.transform.position;
            element.FindPropertyRelative("rotation").quaternionValue = link.transform.rotation;
        }

        private void OnSceneGUI()
        {
            var controller = (UrdfIKController)target;
            if (controller.constraints == null) return;

            foreach (var c in controller.constraints)
            {
                if (c == null || c.type == UrdfIKController.ConstraintType.Joint) continue;
                Vector3 pos = c.Position;
                Quaternion rot = c.Rotation;

                // Handle axes follow the constraint's frame: world, or the target's own axes for Local.
                Quaternion frame = c.frame == UrdfIKController.ConstraintFrame.Local ? rot : Quaternion.identity;

                EditorGUI.BeginChangeCheck();
                Vector3 newPosition = DrawPositionHandles(c, pos, frame);
                Quaternion newRotation = DrawRotationHandles(c, rot, pos, frame);

                Handles.Label(pos, $"IK Target ({c.link})");

                if (EditorGUI.EndChangeCheck())
                {
                    // Record the transform when it's the authority, else the controller holding the stored pose.
                    Undo.RecordObject(c.target != null ? (UnityEngine.Object)c.target : controller, "Move IK Target");
                    c.SetPose(newPosition, newRotation);
                    SceneView.RepaintAll();
                }
            }
        }

        // Only the active position axes get an arrow; all three falls back to the full move handle.
        private static Vector3 DrawPositionHandles(UrdfIKController.IKConstraint c, Vector3 pos, Quaternion frame)
        {
            if (!c.AnyPosition) return pos;
            if (c.posX && c.posY && c.posZ) return Handles.PositionHandle(pos, frame);

            if (c.posX) pos = AxisSlider(pos, frame * Vector3.right, Handles.xAxisColor);
            if (c.posY) pos = AxisSlider(pos, frame * Vector3.up, Handles.yAxisColor);
            if (c.posZ) pos = AxisSlider(pos, frame * Vector3.forward, Handles.zAxisColor);
            return pos;
        }

        // Only the active rotation axes get a disc; all three falls back to the full rotate handle.
        private static Quaternion DrawRotationHandles(UrdfIKController.IKConstraint c, Quaternion rot, Vector3 pos, Quaternion frame)
        {
            if (!c.AnyRotation) return rot;
            if (c.rotX && c.rotY && c.rotZ) return Handles.RotationHandle(rot, pos);

            if (c.rotX) rot = AxisDisc(rot, pos, frame * Vector3.right, Handles.xAxisColor);
            if (c.rotY) rot = AxisDisc(rot, pos, frame * Vector3.up, Handles.yAxisColor);
            if (c.rotZ) rot = AxisDisc(rot, pos, frame * Vector3.forward, Handles.zAxisColor);
            return rot;
        }

        private static Vector3 AxisSlider(Vector3 position, Vector3 direction, Color color)
        {
            using (new Handles.DrawingScope(color))
                return Handles.Slider(position, direction, HandleUtility.GetHandleSize(position), Handles.ArrowHandleCap, 0f);
        }

        private static Quaternion AxisDisc(Quaternion rotation, Vector3 position, Vector3 axis, Color color)
        {
            using (new Handles.DrawingScope(color))
                return Handles.Disc(rotation, position, axis, HandleUtility.GetHandleSize(position), false, 0f);
        }
    }
}
