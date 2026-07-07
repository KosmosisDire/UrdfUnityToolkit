using UnityEditor;
using UnityEngine;
using UrdfToolkit.Urdf;

namespace UrdfToolkit.Editor
{
    // Inspector for UrdfReachBubble: shows the live validity report (can a reach bubble be
    // computed for this robot, and why not), the computed result summary, and the action buttons.
    // (The toolkit's [Button] drawer is scoped to UrdfRobot, so this editor draws its own.)
    [CustomEditor(typeof(UrdfReachBubble))]
    public class UrdfReachBubbleEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            var bubble = (UrdfReachBubble)target;

            EditorGUILayout.Space();

            var report = bubble.Validate();
            foreach (var error in report.errors)
                EditorGUILayout.HelpBox(error, MessageType.Error);
            foreach (var note in report.notes)
                EditorGUILayout.HelpBox(note, MessageType.Info);
            if (report.isValid && !bubble.HasResult)
                EditorGUILayout.HelpBox("Robot qualifies for a reach bubble — press Compute.", MessageType.None);

            if (bubble.HasResult)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Result", EditorStyles.boldLabel);
                EditorGUILayout.LabelField("End effector", bubble.computedForLink);
                EditorGUILayout.LabelField("Max reach", $"{bubble.maxReach:F3} m");
                EditorGUILayout.LabelField("Min reach (inner void)", $"{bubble.minReach:F3} m");
                EditorGUILayout.LabelField("Analytic upper bound", $"{bubble.analyticUpperBound:F3} m");
                EditorGUILayout.LabelField("Confidence (max/bound)", $"{bubble.reachConfidence:P1}");
                if (bubble.HasManipulabilityMap)
                {
                    EditorGUILayout.LabelField("Peak manipulability", $"{bubble.manipPeak:F4}");
                    if (bubble.vertManipPeak > 0f)
                        EditorGUILayout.LabelField("Peak vertical manipulability", $"{bubble.vertManipPeak:F4} m/s per unit q̇");
                }
                if (!bubble.samplingConverged)
                    EditorGUILayout.HelpBox("Max reach was still growing in the second half of the sweep — raise Sample Count and recompute.", MessageType.Warning);
                var q = bubble.CurrentJointValues();
                if (bubble.showSingularityGauge && q != null && bubble.EvaluateConfiguration(q, out _, out float w6, out float wristSin))
                {
                    float wristDeg = Mathf.Asin(Mathf.Clamp01(wristSin)) * Mathf.Rad2Deg;
                    EditorGUILayout.LabelField("Wrist axes angle (now)", $"{wristDeg:F1}°{(wristDeg < bubble.wristWarnAngleDeg ? "  ⚠ near wrist singularity" : "")}");
                    EditorGUILayout.LabelField("Full-pose w6 (now)", $"{w6:F4}");
                }
                if (bubble.HasManipulabilityMap && bubble.showManipulabilitySlice)
                    EditorGUILayout.HelpBox("Manipulability slice: red = near-singular, green = most manipulable, transparent = unreachable. In Combined/Vertical modes, blue = the tool point cannot move vertically there (directional singularity along base Y). The plane follows the tool height — drag the IK target to sweep it.", MessageType.None);
                if (bubble.sliceMode == UrdfReachBubble.SliceMode.OrientationScan)
                    EditorGUILayout.HelpBox(bubble.HasScan
                        ? $"Orientation scan [{bubble.ScanSummary}]: analytic FK samples wrist-aligned to the frozen tool orientation. Blue = wrist-singular there (joint 4 must flip to hold orientation — the 'warble'). Gray = no orientation-matching sample landed there (infeasible at this orientation, or undersampled — raise Sample Count). Transparent = out of reach. The plane is pinned at the scan height; rescan to move it."
                        : "No scan yet this session — position the tool at the height and orientation of interest, then press Scan Slice.", MessageType.None);
                if (bubble.HasManipulabilityMap && bubble.vertManipPeak <= 0f && bubble.sliceMode != UrdfReachBubble.SliceMode.Manipulability)
                    EditorGUILayout.HelpBox("This map was baked before the vertical measure existed (or the chain is rank-deficient) — recompute to enable the Vertical/Combined slice modes.", MessageType.Warning);
            }

            if (bubble.computeManipulabilityMap)
            {
                long voxels = (long)bubble.mapResolution * bubble.mapResolution * bubble.mapResolution;
                double hitPercent = 100.0 * bubble.sampleCount / voxels;
                int solidResolution = Mathf.FloorToInt(Mathf.Pow(bubble.sampleCount, 1f / 3f));
                if (voxels > (long)bubble.sampleCount * 2)
                    EditorGUILayout.HelpBox(
                        $"Map resolution {bubble.mapResolution}³ = {voxels / 1_000_000.0:F0}M voxels, but only {bubble.sampleCount:N0} samples (~{hitPercent:F1}% of voxels hit) — the slice will be sparse speckle. " +
                        $"For solid coverage keep resolution near ∛samples ≈ {solidResolution}, or raise Sample Count with resolution³.", MessageType.Warning);
                if (voxels > UrdfReachBubble.PersistLimitBytes)
                    EditorGUILayout.HelpBox(
                        $"At this resolution the baked maps (~{2 * voxels / 1_000_000.0:F0} MB, plus ~{8 * voxels / 1_000_000.0:F0} MB scratch during compute) are too large to store in the scene — they stay in memory for this session only and must be recomputed after a script reload or scene load.", MessageType.Info);
            }

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!report.isValid))
            {
                if (GUILayout.Button("Compute Reach Bubble"))
                {
                    Undo.RecordObject(bubble, "Compute Reach Bubble");
                    bubble.ComputeReach();
                    EditorUtility.SetDirty(bubble);
                }
            }
            using (new EditorGUI.DisabledScope(!bubble.HasResult))
            {
                if (GUILayout.Button("Scan Slice (Current Orientation)"))
                {
                    try
                    {
                        bubble.ScanSlice(p => EditorUtility.DisplayProgressBar(
                            "Orientation Scan", "Solving IK across the slice plane…", p));
                    }
                    finally
                    {
                        EditorUtility.ClearProgressBar();
                    }
                }
                if (GUILayout.Button("Create Mesh Object"))
                {
                    bubble.CreateMeshObject();
                }
                if (GUILayout.Button("Clear Result"))
                {
                    Undo.RecordObject(bubble, "Clear Reach Bubble");
                    bubble.ClearResults();
                    EditorUtility.SetDirty(bubble);
                }
            }
        }
    }
}
