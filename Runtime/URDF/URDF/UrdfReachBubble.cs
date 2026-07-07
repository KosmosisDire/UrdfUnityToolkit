using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
    /// <summary>
    /// Computes and visualizes the reachable workspace ("reach bubble") of a serial-chain robot.
    ///
    /// Method: Monte-Carlo workspace analysis. Joint configurations are sampled inside the URDF
    /// limits and pushed through the analytical forward kinematics
    /// (<see cref="UrdfJoint.LocalMotionMatrix"/>), so the live robot pose is never touched. The
    /// resulting end-effector positions are binned by direction around the first movable joint
    /// ("shoulder") to build radial envelope meshes — max radius per direction for the outer
    /// envelope, min radius for the inner void.
    ///
    /// Manipulability: the same sweep also fills a 3D voxel map with the best translational
    /// Yoshikawa index w = √det(J·Jᵀ) seen in each voxel (a "capability map"). Manipulability is a
    /// property of a joint configuration, not a position, so per-voxel max answers "how manipulable
    /// can the arm be here" — voxels that only near-singular configurations can reach (workspace
    /// boundary, wrist-over-shoulder lines) stay low. The map is sliced live on the base XZ plane
    /// at the current tool height, so dragging the IK target sweeps the cross-section.
    ///
    /// Applicability: this only makes sense for robots whose workspace is bounded — every prismatic
    /// joint in the chain must have finite limits; revolute joints may be limited or continuous
    /// (continuous is periodic, hence bounded). <see cref="Validate"/> reports exactly why a robot
    /// does or doesn't qualify.
    ///
    /// Confidence: random sampling alone underestimates the true maximum (fully stretched poses are
    /// a measure-zero slice of joint space), so the sweep also evaluates the joint-limit corner
    /// configurations and finishes with coordinate-ascent refinement from the best sample. The
    /// result is checked against the analytic upper bound Σ|link offsets| + prismatic travel:
    /// <see cref="reachConfidence"/> near 1 means the arm can genuinely stretch out straight;
    /// meaningfully lower values are normal for arms with wrist/elbow offsets that can never align.
    /// </summary>
    [RequireComponent(typeof(UrdfRobot))]
    [ExecuteAlways]
    public class UrdfReachBubble : MonoBehaviour
    {
        [Header("Chain")]
        [Tooltip("Link whose origin traces the workspace. Empty = auto-pick the leaf link with the deepest movable chain.")]
        public string endEffectorLink = "";
        [Tooltip("Tool-point offset in the end link's Unity-local frame (e.g. a gripper TCP). NOTE: axes are the imported Unity frame, not the ROS frame — a ROS-convention +Z tool axis is usually local +Y after import.")]
        public Vector3 toolPointOffset = Vector3.zero;
        [Tooltip("Optional Transform marking the tool point (TCP). When set it overrides Tool Point Offset: its position is captured in the end link's frame at compute time and tracked live for the slice. Parent it under the end link so it follows the arm.")]
        public Transform toolPointTransform;

        [Header("Sampling")]
        [Min(1000)] public int sampleCount = 100000;
        public int randomSeed = 12345;
        [Tooltip("Also evaluate every joint-limit corner configuration — extreme reach often lives at the limits.")]
        public bool sampleLimitCorners = true;
        [Tooltip("Coordinate-ascent passes polishing the best sample toward the true maximum reach.")]
        [Range(0, 8)] public int refinementPasses = 3;

        [Header("Envelope")]
        [Range(8, 64)] public int latitudeSegments = 32;
        [Range(16, 128)] public int longitudeSegments = 64;
        [Range(0, 4)] public int smoothingPasses = 1;

        [Header("Manipulability Map")]
        [Tooltip("Fill a 3D voxel map with the best Yoshikawa index per voxel during the sweep.")]
        public bool computeManipulabilityMap = true;
        [Range(16, 512)] public int mapResolution = 40;
        [Tooltip("Show the map cross-section on the base XZ plane at the current tool height.")]
        public bool showManipulabilitySlice = true;
        [Tooltip("Manipulability: Yoshikawa ellipsoid volume (red=singular, green=best). Vertical: directional manipulability along base Y only. Combined: Yoshikawa colors, tinted blue where vertical capability collapses.")]
        public SliceMode sliceMode = SliceMode.Combined;
        [Range(0.05f, 1f)] public float sliceOpacity = 0.6f;

        public enum SliceMode
        {
            Manipulability,          // Yoshikawa index w = σ1·σ2·σ3 (ellipsoid volume)
            VerticalManipulability,  // ellipsoid radius along base-frame Y: 1/√(ŷᵀ(J·Jᵀ)⁻¹ŷ)
            Combined,                // Yoshikawa hue + blue where vertical capability is nearly gone
            OrientationScan,         // IK-solved full-pose scan at a frozen orientation (Scan Slice button)
        }

        [Header("Singularity")]
        [Tooltip("Live gauge at the TCP: the angle between the two outer wrist joint axes (0° = wrist singular, joint 4 must flip) and the full-pose manipulability at the CURRENT configuration.")]
        public bool showSingularityGauge = true;
        [Tooltip("Wrist axes closer than this to collinear count as near-singular: the gauge and scan overlay turn blue.")]
        [Range(1f, 30f)] public float wristWarnAngleDeg = 10f;
        [Tooltip("Grid resolution of the orientation-conditioned IK scan (one IK solve per cell — keep modest).")]
        [Range(16, 96)] public int scanResolution = 32;

        [Header("Visualization")]
        [Tooltip("Only draw the envelopes and manipulability slice while the robot (or one of its children) is selected.")]
        public bool onlyWhenSelected = true;
        public bool drawEnvelope = true;
        public bool drawMinEnvelope = true;
        public bool drawMaxSphere = true;
        public bool drawMinSphere = false;
        public Color envelopeColor = new Color(0f, 0.8f, 1f, 0.25f);
        public Color minEnvelopeColor = new Color(1f, 0.5f, 0f, 0.25f);
        public Color maxSphereColor = new Color(0f, 1f, 0.4f, 0.6f);
        public Color minSphereColor = new Color(1f, 0.5f, 0f, 0.6f);

        // ---- Results (all positions/radii in the base link's local frame, so the bubble follows the robot) ----
        [HideInInspector] public float maxReach;
        [HideInInspector] public float minReach;
        [HideInInspector] public float analyticUpperBound;
        [HideInInspector] public float reachConfidence;   // maxReach / analyticUpperBound
        [HideInInspector] public bool samplingConverged;  // max radius plateaued during the sweep
        [HideInInspector] public Vector3 bubbleCenter;    // first movable joint's pivot, base-local
        [HideInInspector] public string computedForLink;
        [SerializeField, HideInInspector] private float[] envelopeRadii;    // (latitudeSegments+1) x longitudeSegments, max radius
        [SerializeField, HideInInspector] private float[] minEnvelopeRadii; // same grid, min radius
        [SerializeField, HideInInspector] private int radiiLatSegments, radiiLonSegments;
        // The link whose frame the results live in: the chain's first joint's parent (the immovable
        // base) — the same base frame the IK solver's EvaluateFK uses. The bubble follows it.
        [SerializeField, HideInInspector] private UrdfLink frameLink;
        [SerializeField, HideInInspector] private UrdfLink toolLink;
        // The tool offset the results were baked with (end-link local): toolPointOffset, or the
        // captured toolPointTransform position. Keeps the map consistent if the fields change later.
        [SerializeField, HideInInspector] private Vector3 bakedToolOffset;

        // Manipulability voxel map: cube of half-extent mapExtent centered on bubbleCenter, base-local.
        // Bytes: 0 = no sample reached the voxel, 1..255 = best w seen, normalized by manipPeak.
        // The working maps are deliberately NOT serialized: at high resolutions they reach tens of
        // MB, and any [SerializeField] of that size is re-serialized on every inspector repaint,
        // undo snapshot and scene save — the editor grinds continuously long after the compute
        // finished. Maps up to PersistLimitBytes are mirrored into the persisted fields so small
        // bakes survive reloads; larger ones are session-only and must be recomputed.
        [System.NonSerialized] private byte[] manipMap;
        [System.NonSerialized] private byte[] vertManipMap; // same layout: directional manipulability along base Y
        [SerializeField, HideInInspector] private byte[] persistedManipMap;
        [SerializeField, HideInInspector] private byte[] persistedVertMap;
        [SerializeField, HideInInspector] private int manipRes;
        [SerializeField, HideInInspector] private float mapExtent;

        public const int PersistLimitBytes = 2 * 1024 * 1024;
        [HideInInspector] public float manipPeak;     // w value a manipMap byte of 255 corresponds to
        [HideInInspector] public float vertManipPeak; // m_y value a vertManipMap byte of 255 corresponds to

        [System.NonSerialized] private Mesh envelopeMesh;
        [System.NonSerialized] private Mesh minEnvelopeMesh;

        // Compute-time scratch (allocated per ComputeReach call, null otherwise).
        [System.NonSerialized] private float[] manipAccum;
        [System.NonSerialized] private float[] vertAccum;
        [System.NonSerialized] private Vector3[] samplePivots;
        [System.NonSerialized] private Vector3[] sampleAxes;
        [System.NonSerialized] private bool[] movableIsPrismatic;

        // Live slice objects (never saved; rebuilt on demand).
        [System.NonSerialized] private GameObject sliceObject;
        [System.NonSerialized] private Texture2D sliceTexture;
        [System.NonSerialized] private float sliceY = float.NaN;
        [System.NonSerialized] private SliceMode lastSliceMode;

        // Orientation-conditioned scan results (session-only; invalidated by ComputeReach).
        [System.NonSerialized] private float[] scanW;        // full-pose w per cell; -1 = out of reach, -2 = orientation-infeasible
        [System.NonSerialized] private float[] scanWristSin; // |sin(outer wrist axes angle)| per cell
        [System.NonSerialized] private int scanRes;
        [System.NonSerialized] private float scanYLocal;
        [System.NonSerialized] private float scanWMax;
        [System.NonSerialized] private bool hasScan;
        [System.NonSerialized] private string scanSummary;
        [System.NonSerialized] private UrdfJoint[] cachedChain;

        public bool HasScan => hasScan;
        public string ScanSummary => scanSummary;

        public bool HasResult => envelopeRadii != null && envelopeRadii.Length > 0;

        public bool HasManipulabilityMap
        {
            get
            {
                // Restore a persisted (small) map after a domain reload / scene load.
                if (manipMap == null && persistedManipMap != null && persistedManipMap.Length > 0)
                {
                    manipMap = persistedManipMap;
                    vertManipMap = persistedVertMap != null && persistedVertMap.Length > 0 ? persistedVertMap : null;
                }
                return manipMap != null && manipMap.Length > 0;
            }
        }

        private const string SliceObjectName = "~ReachManipulabilitySlice";

        // ---------------------------------------------------------------- Validation

        public class ValidationReport
        {
            public bool isValid;
            public UrdfLink endLink;
            public List<UrdfJoint> chain = new List<UrdfJoint>();     // root -> end, fixed included
            public List<UrdfJoint> movable = new List<UrdfJoint>();   // sampled joints, chain order
            public List<string> errors = new List<string>();
            public List<string> notes = new List<string>();
        }

        /// <summary>
        /// Checks whether a reach bubble is computable and explains why (not). The criteria:
        /// a serial chain from the base to one end link, at least one movable joint, and a bounded
        /// joint space — finite limits on every prismatic joint; revolute joints limited or continuous.
        /// </summary>
        public ValidationReport Validate()
        {
            var report = new ValidationReport();
            var links = GetComponentsInChildren<UrdfLink>();

            if (links.Length == 0)
            {
                report.errors.Add("No UrdfLink components found under this robot.");
                return report;
            }

            report.endLink = ResolveEndLink(links, report);
            if (report.endLink == null) return report;

            // Walk the link->joint->parentLink chain up to the base. This is the serial-chain
            // requirement made concrete: exactly one path from the end link to the root.
            var link = report.endLink;
            while (link != null && link.joint != null)
            {
                report.chain.Insert(0, link.joint);
                link = link.joint.parentLink;
                if (report.chain.Count > links.Length)
                {
                    report.errors.Add("Joint chain does not terminate at a base link (cycle or broken parent references).");
                    return report;
                }
            }

            report.movable = report.chain.Where(j => !j.IsFixed).ToList();
            if (report.movable.Count == 0)
            {
                report.errors.Add($"Chain to '{report.endLink.name}' has no movable joints — the robot is static along this chain.");
                return report;
            }

            int revolute = 0, continuous = 0, prismatic = 0;
            foreach (var joint in report.movable)
            {
                if (joint is UrdfJointRevolute)
                {
                    if (joint.HasLimits)
                    {
                        if (joint.UpperLimit < joint.LowerLimit)
                            report.errors.Add($"Revolute joint '{joint.JointName}' has inverted limits ({joint.LowerLimit:F1} > {joint.UpperLimit:F1}).");
                        revolute++;
                    }
                    else
                    {
                        continuous++; // periodic — bounded reach even without limits
                    }
                }
                else if (joint is UrdfJointPrismatic)
                {
                    if (!joint.HasLimits)
                        report.errors.Add($"Prismatic joint '{joint.JointName}' has no finite limits — the workspace is unbounded, no reach bubble exists.");
                    else
                        prismatic++;
                }
                else
                {
                    report.errors.Add($"Joint '{joint.JointName}' has unsupported type {joint.GetType().Name}.");
                }
            }

            report.notes.Add($"Chain to '{report.endLink.name}': {revolute} limited revolute, {continuous} continuous, {prismatic} prismatic, {report.chain.Count - report.movable.Count} fixed.");
            if (revolute + continuous == 0)
                report.notes.Add("Chain is purely prismatic (gantry-style) — the workspace is a box, not a bubble; the envelope still visualizes it correctly.");
            if (report.movable.Count < 3 && computeManipulabilityMap)
                report.notes.Add($"Only {report.movable.Count} DOF: the 3D translational Jacobian is rank-deficient everywhere, so the manipulability map will be uniformly singular (zero).");
            if (report.movable.Count > 12)
                report.notes.Add($"{report.movable.Count} DOF is high for Monte-Carlo coverage — expect an underestimated envelope; raise the sample count.");

            report.isValid = report.errors.Count == 0;
            return report;
        }

        private UrdfLink ResolveEndLink(UrdfLink[] links, ValidationReport report)
        {
            if (!string.IsNullOrEmpty(endEffectorLink))
            {
                var named = links.FirstOrDefault(l => l.name == endEffectorLink);
                if (named == null)
                    report.errors.Add($"End-effector link '{endEffectorLink}' not found on this robot.");
                return named;
            }

            // Auto: the leaf link with the most movable joints between it and the base.
            var leaves = links.Where(l => l.childLinks == null || l.childLinks.Count == 0).ToList();
            if (leaves.Count == 0) leaves = links.ToList();

            UrdfLink best = null;
            int bestDepth = -1, tiedAtBest = 0;
            foreach (var leaf in leaves)
            {
                int depth = 0;
                var link = leaf;
                int guard = links.Length + 1;
                while (link != null && link.joint != null && guard-- > 0)
                {
                    if (!link.joint.IsFixed) depth++;
                    link = link.joint.parentLink;
                }
                if (depth > bestDepth) { best = leaf; bestDepth = depth; tiedAtBest = 1; }
                else if (depth == bestDepth) tiedAtBest++;
            }

            if (best != null)
            {
                report.notes.Add($"Auto-selected end effector: '{best.name}' ({bestDepth} movable joints deep).");
                if (tiedAtBest > 1)
                    report.notes.Add($"{tiedAtBest} leaf links tie at that depth (branching robot?) — set End Effector Link explicitly to pick the right one.");
            }
            return best;
        }

        // ---------------------------------------------------------------- Computation

        public void ComputeReach()
        {
            var report = Validate();
            foreach (var note in report.notes) Debug.Log($"[reach] {note}", this);
            if (!report.isValid)
            {
                foreach (var error in report.errors) Debug.LogError($"[reach] {error}", this);
                return;
            }

            var chain = report.chain.ToArray();
            var movable = report.movable.ToArray();
            var (lower, upper) = SamplingRanges(movable);
            frameLink = chain[0].parentLink; // base frame all results are expressed in
            toolLink = report.endLink;
            cachedChain = null;  // tool link may have changed
            hasScan = false;     // scan geometry (extent/height) is tied to the previous bake
            bakedToolOffset = toolPointTransform != null
                ? report.endLink.transform.InverseTransformPoint(toolPointTransform.position)
                : toolPointOffset;

            // Shoulder pivot: everything before the first movable joint is rigid, so this point is
            // fixed in the base frame and is the natural center to measure reach from.
            int firstMovable = System.Array.IndexOf(chain, movable[0]);
            var prefix = Matrix4x4.identity;
            for (int i = 0; i <= firstMovable; i++)
                prefix *= chain[i].LocalMotionMatrix(0f);
            bubbleCenter = prefix.GetColumn(3);

            analyticUpperBound = AnalyticUpperBound(chain, firstMovable);

            radiiLatSegments = latitudeSegments;
            radiiLonSegments = longitudeSegments;
            envelopeRadii = new float[(latitudeSegments + 1) * longitudeSegments];
            minEnvelopeRadii = new float[envelopeRadii.Length];
            for (int i = 0; i < minEnvelopeRadii.Length; i++) minEnvelopeRadii[i] = float.PositiveInfinity;
            maxReach = 0f;
            minReach = float.PositiveInfinity;

            var values = new float[movable.Length];
            var bestValues = new float[movable.Length];
            var rng = new System.Random(randomSeed);

            // Joint-limit corners: extreme reach configurations usually sit at the limits.
            void SampleCorners()
            {
                if (!sampleLimitCorners) return;
                int corners = movable.Length <= 10 ? 1 << movable.Length : 1024;
                for (int c = 0; c < corners; c++)
                {
                    for (int i = 0; i < values.Length; i++)
                    {
                        bool high = movable.Length <= 10 ? ((c >> i) & 1) == 1 : rng.Next(2) == 1;
                        values[i] = high ? upper[i] : lower[i];
                    }
                    if (RecordSample(chain, values) >= maxReach) values.CopyTo(bestValues, 0);
                }
            }

            // Coordinate-ascent polish of the best configuration: scan each joint over its range
            // while holding the others, repeat. Closes the gap between "best sample so far" and the
            // true stretched-out maximum.
            void RefineBest()
            {
                const int scanSteps = 24;
                for (int pass = 0; pass < refinementPasses; pass++)
                {
                    for (int i = 0; i < movable.Length; i++)
                    {
                        bestValues.CopyTo(values, 0);
                        float bestValue = bestValues[i];
                        for (int step = 0; step <= scanSteps; step++)
                        {
                            values[i] = Mathf.Lerp(lower[i], upper[i], step / (float)scanSteps);
                            if (RecordSample(chain, values) >= maxReach) bestValue = values[i];
                        }
                        bestValues[i] = bestValue;
                    }
                }
            }

            // Pre-size pass: corners + refinement measure (nearly) the true max reach in a few
            // thousand FK evaluations, so the voxel grid can wrap the actual workspace instead of
            // the loose analytic bound — full resolution where samples can land, and the slice quad
            // hugs the reach. manipAccum is still null here, so these samples only feed the
            // envelope bins and max/min; the same stages run again after the grid exists.
            if (computeManipulabilityMap)
            {
                SampleCorners();
                RefineBest();

                manipRes = mapResolution;
                mapExtent = maxReach > 1e-4f ? maxReach * 1.02f : analyticUpperBound;
                manipAccum = new float[manipRes * manipRes * manipRes];
                vertAccum = new float[manipAccum.Length];
                samplePivots = new Vector3[movable.Length];
                sampleAxes = new Vector3[movable.Length];
                movableIsPrismatic = movable.Select(j => j is UrdfJointPrismatic).ToArray();
                manipPeak = 0f;
                vertManipPeak = 0f;
            }
            else
            {
                manipAccum = null;
                vertAccum = null;
                manipMap = null;
                vertManipMap = null;
                persistedManipMap = null;
                persistedVertMap = null;
            }

            // Random sweep. Convergence is judged on the random samples alone (the pre-pass has
            // usually saturated maxReach already, which would make a maxReach-based check vacuous).
            float randomMax = 0f, randomMaxAtHalf = 0f;
            for (int s = 0; s < sampleCount; s++)
            {
                for (int i = 0; i < values.Length; i++)
                    values[i] = Mathf.Lerp(lower[i], upper[i], (float)rng.NextDouble());
                float radius = RecordSample(chain, values);
                if (radius > randomMax) randomMax = radius;
                if (radius >= maxReach) values.CopyTo(bestValues, 0);
                if (s == sampleCount / 2) randomMaxAtHalf = randomMax;
            }
            samplingConverged = randomMax > 0f && (randomMax - randomMaxAtHalf) <= 0.005f * randomMax;

            // Corners + refinement (again, when the pre-pass already ran them): with the grid now
            // allocated, these extreme configurations also deposit into the manipulability map.
            SampleCorners();
            RefineBest();

            if (minReach > maxReach) minReach = 0f;
            reachConfidence = analyticUpperBound > 1e-6f ? maxReach / analyticUpperBound : 0f;
            computedForLink = report.endLink.name;

            envelopeRadii = FillHoles(envelopeRadii, 0f);
            minEnvelopeRadii = FillHoles(minEnvelopeRadii, float.PositiveInfinity);
            for (int i = 0; i < minEnvelopeRadii.Length; i++)
                if (float.IsInfinity(minEnvelopeRadii[i])) minEnvelopeRadii[i] = 0f; // nothing sampled anywhere
            for (int i = 0; i < smoothingPasses; i++)
            {
                envelopeRadii = Smooth(envelopeRadii);
                minEnvelopeRadii = Smooth(minEnvelopeRadii);
            }
            envelopeMesh = null;    // rebuild lazily from the new radii
            minEnvelopeMesh = null;

            QuantizeManipMap();
            manipAccum = null;
            vertAccum = null;
            samplePivots = sampleAxes = null;
            movableIsPrismatic = null;
            DestroySlice(); // rebuilt next Update from the new map

            Debug.Log($"[reach] '{computedForLink}': max {maxReach:F3} m, min {minReach:F3} m, " +
                      $"analytic bound {analyticUpperBound:F3} m (confidence {reachConfidence:P0}), " +
                      $"peak manipulability {manipPeak:F4}, " +
                      $"sampling {(samplingConverged ? "converged" : "NOT converged — raise Sample Count")}.", this);
        }

        public void ClearResults()
        {
            envelopeRadii = null;
            minEnvelopeRadii = null;
            envelopeMesh = null;
            minEnvelopeMesh = null;
            manipMap = null;
            vertManipMap = null;
            persistedManipMap = null;
            persistedVertMap = null;
            maxReach = minReach = analyticUpperBound = reachConfidence = manipPeak = vertManipPeak = 0f;
            samplingConverged = false;
            computedForLink = null;
            hasScan = false;
            scanW = null;
            scanWristSin = null;
            cachedChain = null;
            DestroySlice();
        }

        /// <summary>Sampling range per movable joint: URDF limits, or a full turn for continuous joints.</summary>
        private static (float[] lower, float[] upper) SamplingRanges(UrdfJoint[] movable)
        {
            var lower = new float[movable.Length];
            var upper = new float[movable.Length];
            for (int i = 0; i < movable.Length; i++)
            {
                if (movable[i].HasLimits)
                {
                    lower[i] = (float)movable[i].LowerLimit;
                    upper[i] = (float)movable[i].UpperLimit;
                }
                else
                {
                    lower[i] = -180f; // continuous revolute — Validate() rejects unlimited prismatic
                    upper[i] = 180f;
                }
            }
            return (lower, upper);
        }

        /// <summary>
        /// Rigid-body upper bound on reach from the shoulder pivot: the chain can never stretch
        /// further than the sum of its inter-joint offsets plus prismatic travel plus the tool
        /// offset (triangle inequality — rotations preserve segment lengths).
        /// </summary>
        private float AnalyticUpperBound(UrdfJoint[] chain, int firstMovable)
        {
            float bound = bakedToolOffset.magnitude;
            for (int i = firstMovable; i < chain.Length; i++)
            {
                if (i > firstMovable)
                    bound += chain[i].LocalMotionMatrix(0f).GetColumn(3).magnitude;
                if (chain[i] is UrdfJointPrismatic && chain[i].HasLimits)
                    bound += Mathf.Max(Mathf.Abs((float)chain[i].LowerLimit), Mathf.Abs((float)chain[i].UpperLimit));
            }
            return bound;
        }

        /// <summary>
        /// FK for one configuration; records the point into max/min reach, the direction bins, and
        /// (when enabled) the manipulability voxel map. Returns the radius from the bubble center.
        /// </summary>
        private float RecordSample(UrdfJoint[] chain, float[] movableValues)
        {
            var m = Matrix4x4.identity;
            int vi = 0;
            foreach (var joint in chain)
            {
                bool isMovable = !joint.IsFixed;
                m *= joint.LocalMotionMatrix(isMovable ? movableValues[vi] : 0f);
                if (isMovable)
                {
                    if (manipAccum != null)
                    {
                        // The joint's own motion doesn't move its frame origin (revolute) or its
                        // axis direction (both types), so sampling AFTER applying the joint is valid.
                        samplePivots[vi] = m.GetColumn(3);
                        sampleAxes[vi] = m.MultiplyVector(joint.LocalAxis).normalized;
                    }
                    vi++;
                }
            }
            Vector3 point = m.MultiplyPoint3x4(bakedToolOffset);

            Vector3 d = point - bubbleCenter;
            float radius = d.magnitude;
            if (radius > maxReach) maxReach = radius;
            if (radius < minReach) minReach = radius;

            if (radius > 1e-6f)
            {
                float theta = Mathf.Acos(Mathf.Clamp(d.y / radius, -1f, 1f));   // 0..π from +Y
                float phi = Mathf.Atan2(d.z, d.x);                              // -π..π around Y
                int lat = Mathf.Clamp(Mathf.RoundToInt(theta / Mathf.PI * radiiLatSegments), 0, radiiLatSegments);
                int lon = Mathf.RoundToInt((phi + Mathf.PI) / (2f * Mathf.PI) * radiiLonSegments) % radiiLonSegments;
                int index = lat * radiiLonSegments + lon;
                if (radius > envelopeRadii[index]) envelopeRadii[index] = radius;
                if (radius < minEnvelopeRadii[index]) minEnvelopeRadii[index] = radius;
            }

            if (manipAccum != null) RecordManipulability(point, vi);
            return radius;
        }

        /// <summary>
        /// Translational Yoshikawa index w = √det(J·Jᵀ) for the configuration captured by
        /// <see cref="RecordSample"/>, deposited into the voxel containing the tool point (keeping
        /// the per-voxel max). Jacobian columns: revolute ω̂×(p−pᵢ) (per radian), prismatic â.
        /// </summary>
        private void RecordManipulability(Vector3 point, int movableCount)
        {
            float axx = 0, axy = 0, axz = 0, ayy = 0, ayz = 0, azz = 0; // A = J·Jᵀ, symmetric 3x3
            for (int i = 0; i < movableCount; i++)
            {
                Vector3 col = movableIsPrismatic[i]
                    ? sampleAxes[i]
                    : Vector3.Cross(sampleAxes[i], point - samplePivots[i]);
                axx += col.x * col.x; axy += col.x * col.y; axz += col.x * col.z;
                ayy += col.y * col.y; ayz += col.y * col.z;
                azz += col.z * col.z;
            }
            float det = axx * (ayy * azz - ayz * ayz)
                      - axy * (axy * azz - ayz * axz)
                      + axz * (axy * ayz - ayy * axz);
            float w = Mathf.Sqrt(Mathf.Max(0f, det));

            // Directional manipulability along base Y: the ellipsoid radius m_y = 1/√(ŷᵀA⁻¹ŷ).
            // By Cramer's rule (A⁻¹)₁₁ = (AxxAzz − Axz²)/det, so m_y = √(det / (AxxAzz − Axz²)) —
            // the achievable vertical tool speed per unit joint-velocity norm. → 0 when vertical
            // motion leaves the range of J (a directional singularity), even if w is still healthy.
            float yCofactor = axx * azz - axz * axz;
            float my = det > 1e-12f && yCofactor > 1e-12f ? Mathf.Sqrt(det / yCofactor) : 0f;

            float cell = 2f * mapExtent / manipRes;
            Vector3 g = (point - bubbleCenter + Vector3.one * mapExtent) / cell;
            int gx = (int)g.x, gy = (int)g.y, gz = (int)g.z;
            if (gx < 0 || gx >= manipRes || gy < 0 || gy >= manipRes || gz < 0 || gz >= manipRes) return;

            int index = (gy * manipRes + gz) * manipRes + gx;
            if (w > manipAccum[index]) manipAccum[index] = w;
            if (w > manipPeak) manipPeak = w;
            if (my > vertAccum[index]) vertAccum[index] = my;
            if (my > vertManipPeak) vertManipPeak = my;
        }

        // Normalize the accumulated values into bytes: 0 = voxel never reached (or exactly singular
        // for the directional map), 1..255 = value/peak.
        private void QuantizeManipMap()
        {
            if (manipAccum == null) return;
            manipMap = Quantize(manipAccum, manipPeak);
            vertManipMap = Quantize(vertAccum, vertManipPeak);

            // Only small maps go into the scene file (see the field comment); big ones stay in memory.
            bool persist = manipMap.Length <= PersistLimitBytes;
            persistedManipMap = persist ? manipMap : null;
            persistedVertMap = persist ? vertManipMap : null;
        }

        private static byte[] Quantize(float[] accum, float peak)
        {
            var map = new byte[accum.Length];
            if (peak <= 0f) return map; // rank-deficient chain: all reached voxels stay byte 0-equivalent
            for (int i = 0; i < accum.Length; i++)
                if (accum[i] > 0f)
                    map[i] = (byte)Mathf.Clamp(1 + Mathf.RoundToInt(accum[i] / peak * 254f), 1, 255);
            return map;
        }

        // ---------------------------------------------------------------- Full-pose singularity

        // The chain root→tool link, cached for per-frame gauge evaluation (invalidated on compute).
        private UrdfJoint[] CurrentChain()
        {
            if (cachedChain != null) return cachedChain;
            var end = ToolLink();
            if (end == null) return null;
            var list = new List<UrdfJoint>();
            var link = end;
            int guard = 256;
            while (link != null && link.joint != null && guard-- > 0)
            {
                list.Insert(0, link.joint);
                link = link.joint.parentLink;
            }
            cachedChain = list.ToArray();
            return cachedChain;
        }

        /// <summary>Current values of the chain's movable joints, in chain order (solver-compatible).</summary>
        public float[] CurrentJointValues()
        {
            var chain = CurrentChain();
            return chain?.Where(j => !j.IsFixed).Select(j => j.GetPosition()).ToArray();
        }

        /// <summary>
        /// Full-pose singularity measures for one configuration (movable values in chain order):
        /// <paramref name="w6"/> = √det(J·Jᵀ) of the full 6×n Jacobian (translation rows scaled by
        /// 1/maxReach so metres and radians mix consistently; needs ≥6 DOF, else 0), and
        /// <paramref name="wristSin"/> = |sin(angle between the outer wrist axes n−3 and n−1)| —
        /// 0 when collinear, the classic wrist singularity where joint 4 must flip 180° to hold
        /// orientation. The translational map can NEVER show that one: position motion stays fine
        /// there; it is the orientation constraint that becomes impossible to track smoothly.
        /// </summary>
        public bool EvaluateConfiguration(float[] movableValues, out Vector3 tcpLocal, out float w6, out float wristSin)
        {
            tcpLocal = default;
            w6 = 0f;
            wristSin = 1f;
            var chain = CurrentChain();
            if (chain == null || chain.Length == 0) return false;

            int n = chain.Count(j => !j.IsFixed);
            if (movableValues == null || movableValues.Length < n) return false;
            var axes = new Vector3[n];
            var pivots = new Vector3[n];
            var isPrismatic = new bool[n];

            var m = Matrix4x4.identity;
            int vi = 0;
            foreach (var joint in chain)
            {
                bool isMovable = !joint.IsFixed;
                m *= joint.LocalMotionMatrix(isMovable ? movableValues[vi] : 0f);
                if (isMovable)
                {
                    pivots[vi] = m.GetColumn(3);
                    axes[vi] = m.MultiplyVector(joint.LocalAxis).normalized;
                    isPrismatic[vi] = joint is UrdfJointPrismatic;
                    vi++;
                }
            }
            tcpLocal = m.MultiplyPoint3x4(bakedToolOffset);

            if (n >= 3 && !isPrismatic[n - 1] && !isPrismatic[n - 3])
                wristSin = Vector3.Cross(axes[n - 3], axes[n - 1]).magnitude;

            if (n >= 6)
            {
                float charLen = maxReach > 1e-3f ? maxReach : 1f;
                var A = new float[6, 6]; // J·Jᵀ, symmetric
                for (int i = 0; i < n; i++)
                {
                    Vector3 lin = (isPrismatic[i] ? axes[i] : Vector3.Cross(axes[i], tcpLocal - pivots[i])) / charLen;
                    Vector3 rot = isPrismatic[i] ? Vector3.zero : axes[i];
                    float[] col = { lin.x, lin.y, lin.z, rot.x, rot.y, rot.z };
                    for (int r = 0; r < 6; r++)
                    for (int c = r; c < 6; c++)
                        A[r, c] += col[r] * col[c];
                }
                for (int r = 1; r < 6; r++)
                for (int c = 0; c < r; c++)
                    A[r, c] = A[c, r];
                w6 = Mathf.Sqrt(Mathf.Max(0f, Det6(A)));
            }
            return true;
        }

        // Determinant of a 6×6 via Gaussian elimination with partial pivoting (on a copy).
        private static float Det6(float[,] a)
        {
            const int N = 6;
            var mtx = (float[,])a.Clone();
            float det = 1f;
            for (int i = 0; i < N; i++)
            {
                int pivot = i;
                for (int r = i + 1; r < N; r++)
                    if (Mathf.Abs(mtx[r, i]) > Mathf.Abs(mtx[pivot, i])) pivot = r;
                if (Mathf.Abs(mtx[pivot, i]) < 1e-12f) return 0f;
                if (pivot != i)
                {
                    for (int c = 0; c < N; c++)
                        (mtx[i, c], mtx[pivot, c]) = (mtx[pivot, c], mtx[i, c]);
                    det = -det;
                }
                det *= mtx[i, i];
                for (int r = i + 1; r < N; r++)
                {
                    float f = mtx[r, i] / mtx[i, i];
                    for (int c = i; c < N; c++) mtx[r, c] -= f * mtx[i, c];
                }
            }
            return det;
        }

        /// <summary>
        /// Orientation-conditioned singularity scan of the slice plane — pure analytic FK, the same
        /// calculation as the live gauge, no IK solver involved. Random configurations are drawn as
        /// in the main sweep; for each, the three wrist joints are re-solved (small damped Newton on
        /// the 3×3 rotational Jacobian) to match the CURRENT tool orientation, and matching samples
        /// near the slice height are measured and binned. This surfaces the singularities the
        /// per-voxel-max map integrates away: on the wrist-singular surface wristSin ≈ 0 in every
        /// matching configuration, so the blue locus is branch-independent. Session-only; shown with
        /// Slice Mode = OrientationScan (set automatically when the scan finishes).
        /// </summary>
        public void ScanSlice(System.Action<float> progress = null)
        {
            var frame = FrameLink();
            var tool = ToolLink();
            if (frame == null || tool == null || !HasResult)
            {
                Debug.LogWarning("[reach] Compute the reach bubble before scanning.", this);
                return;
            }
            var report = Validate();
            if (!report.isValid)
            {
                foreach (var error in report.errors) Debug.LogError($"[reach] {error}", this);
                return;
            }
            var chain = report.chain.ToArray();
            var movable = report.movable.ToArray();
            int n = movable.Length;
            if (n < 3 || movable[n - 1] is UrdfJointPrismatic || movable[n - 2] is UrdfJointPrismatic || movable[n - 3] is UrdfJointPrismatic)
            {
                Debug.LogWarning("[reach] Orientation scan needs three terminal revolute (wrist) joints to align the tool orientation.", this);
                return;
            }
            var (lower, upper) = SamplingRanges(movable);

            int res = scanResolution;
            float extent = mapExtent > 0f ? mapExtent : maxReach;
            float cellSize = 2f * extent / res;
            Vector3 toolLocal = frame.transform.InverseTransformPoint(ToolPointWorld());
            float ySlice = Mathf.Clamp(toolLocal.y, bubbleCenter.y - extent, bubbleCenter.y + extent);
            // Reference orientation in the base frame — all FK here is base-local.
            Quaternion refRot = Quaternion.Inverse(frame.transform.rotation) * tool.transform.rotation;

            scanW = new float[res * res];
            scanWristSin = new float[res * res];
            scanRes = res;
            scanYLocal = ySlice;
            scanWMax = 0f;

            // Pre-classify cells: out of reach vs "no matching sample" (gray until data lands).
            for (int cz = 0; cz < res; cz++)
            for (int cx = 0; cx < res; cx++)
            {
                var local = new Vector3(
                    bubbleCenter.x - extent + (cx + 0.5f) * cellSize,
                    ySlice,
                    bubbleCenter.z - extent + (cz + 0.5f) * cellSize);
                scanW[cz * res + cx] = EnvelopeReachable(local) ? -2f : -1f;
                scanWristSin[cz * res + cx] = 1f;
            }

            // Aligning the wrist moves the TCP by at most the wrist→TCP length; pre-filter samples
            // by height with that slack before paying for the (much costlier) alignment iteration.
            float wristRadius = bakedToolOffset.magnitude;
            for (int i = System.Array.IndexOf(chain, movable[n - 3]) + 1; i < chain.Length; i++)
                wristRadius += ((Vector3)chain[i].LocalMotionMatrix(0f).GetColumn(3)).magnitude;
            float slabHalf = cellSize; // half-thickness of the height band binned into the slice

            var rng = new System.Random(randomSeed + 1);
            var q = new float[n];
            var timer = System.Diagnostics.Stopwatch.StartNew();
            int deposited = 0;
            for (int s = 0; s < sampleCount; s++)
            {
                if ((s & 8191) == 0) progress?.Invoke(s / (float)sampleCount);
                for (int i = 0; i < n; i++)
                    q[i] = Mathf.Lerp(lower[i], upper[i], (float)rng.NextDouble());

                Vector3 rough = FKPoint(chain, q);
                if (Mathf.Abs(rough.y - ySlice) > slabHalf + 2f * wristRadius) continue;

                if (!AlignWrist(chain, movable, q, refRot, lower, upper)) continue;
                if (!EvaluateConfiguration(q, out Vector3 p, out float w6, out float wristSin)) continue;
                if (Mathf.Abs(p.y - ySlice) > slabHalf) continue;

                int cx = (int)((p.x - (bubbleCenter.x - extent)) / cellSize);
                int cz = (int)((p.z - (bubbleCenter.z - extent)) / cellSize);
                if (cx < 0 || cx >= res || cz < 0 || cz >= res) continue;
                int cell = cz * res + cx;
                if (scanW[cell] < 0f) scanW[cell] = 0f;
                if (w6 > scanW[cell]) scanW[cell] = w6;
                if (w6 > scanWMax) scanWMax = w6;
                if (wristSin < scanWristSin[cell]) scanWristSin[cell] = wristSin;
                deposited++;
            }
            progress?.Invoke(1f);

            int filled = scanW.Count(v => v >= 0f);
            int gray = scanW.Count(v => v == -2f);
            hasScan = true;
            sliceMode = SliceMode.OrientationScan;
            sliceY = float.NaN; // force retexture
            scanSummary = $"{deposited} samples into {filled} cells, {gray} cells without a matching sample ({timer.Elapsed.TotalSeconds:F1}s)";
            Debug.Log($"[reach] Orientation scan {res}×{res} at base height {ySlice:F3} m: {scanSummary}. " +
                      "Blue = wrist-singular at this orientation (the joint-4 flip locus).", this);

            // Build/retexture the slice NOW: in edit mode nothing else pumps Update() after an
            // inspector button press, so without this the scan stays invisible until some
            // unrelated interaction happens to tick the player loop.
            Update();
#if UNITY_EDITOR
            UnityEditor.SceneView.RepaintAll();
#endif
        }

        // Plain FK: base-local TCP position for a configuration (movable values in chain order).
        private Vector3 FKPoint(UrdfJoint[] chain, float[] q)
        {
            var m = Matrix4x4.identity;
            int vi = 0;
            foreach (var joint in chain)
                m *= joint.LocalMotionMatrix(joint.IsFixed ? 0f : q[vi++]);
            return m.MultiplyPoint3x4(bakedToolOffset);
        }

        /// <summary>
        /// Rotates the last three (wrist) joints in place so the tool orientation reaches
        /// <paramref name="refRot"/> (base frame): damped Newton on the 3×3 rotational Jacobian
        /// whose columns are the wrist axes — the same analytic FK the live gauge uses. Respects
        /// joint limits (clamped; continuous joints wrap). Returns true when within ~3°.
        /// </summary>
        private bool AlignWrist(UrdfJoint[] chain, UrdfJoint[] movable, float[] q, Quaternion refRot, float[] lower, float[] upper)
        {
            int n = q.Length;
            const float lambda = 0.1f; // damping — also keeps the step finite AT the wrist singularity
            for (int iter = 0; iter < 10; iter++)
            {
                // FK for the current tool rotation and the three wrist axes (base frame).
                var m = Matrix4x4.identity;
                int vi = 0;
                Vector3 a0 = Vector3.right, a1 = Vector3.up, a2 = Vector3.forward;
                foreach (var joint in chain)
                {
                    bool isMovable = !joint.IsFixed;
                    m *= joint.LocalMotionMatrix(isMovable ? q[vi] : 0f);
                    if (isMovable)
                    {
                        if (vi == n - 3) a0 = m.MultiplyVector(joint.LocalAxis).normalized;
                        else if (vi == n - 2) a1 = m.MultiplyVector(joint.LocalAxis).normalized;
                        else if (vi == n - 1) a2 = m.MultiplyVector(joint.LocalAxis).normalized;
                        vi++;
                    }
                }

                (refRot * Quaternion.Inverse(m.rotation)).ToAngleAxis(out float angleDeg, out Vector3 axis);
                if (angleDeg > 180f) angleDeg -= 360f;
                if (Mathf.Abs(angleDeg) < 3f) return true;
                Vector3 err = axis.normalized * (angleDeg * Mathf.Deg2Rad); // rotation vector

                // Damped least squares Δ = Jᵀ(J·Jᵀ + λ²I)⁻¹ err with J = [a0 a1 a2] as columns.
                // J·Jᵀ = Σ aᵢaᵢᵀ; 3×3 inverse by Cramer's rule.
                float xx = a0.x * a0.x + a1.x * a1.x + a2.x * a2.x + lambda * lambda;
                float yy = a0.y * a0.y + a1.y * a1.y + a2.y * a2.y + lambda * lambda;
                float zz = a0.z * a0.z + a1.z * a1.z + a2.z * a2.z + lambda * lambda;
                float xy = a0.x * a0.y + a1.x * a1.y + a2.x * a2.y;
                float xz = a0.x * a0.z + a1.x * a1.z + a2.x * a2.z;
                float yz = a0.y * a0.z + a1.y * a1.z + a2.y * a2.z;
                float det = xx * (yy * zz - yz * yz) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz);
                if (Mathf.Abs(det) < 1e-12f) return false;
                var x = new Vector3(
                    ((yy * zz - yz * yz) * err.x + (xz * yz - xy * zz) * err.y + (xy * yz - xz * yy) * err.z) / det,
                    ((yz * xz - xy * zz) * err.x + (xx * zz - xz * xz) * err.y + (xy * xz - xx * yz) * err.z) / det,
                    ((xy * yz - yy * xz) * err.x + (xy * xz - xx * yz) * err.y + (xx * yy - xy * xy) * err.z) / det);

                for (int w = 0; w < 3; w++)
                {
                    int ji = n - 3 + w;
                    Vector3 column = w == 0 ? a0 : w == 1 ? a1 : a2;
                    float next = q[ji] + Vector3.Dot(column, x) * Mathf.Rad2Deg;
                    // Continuous joints wrap; limited ones clamp (limits are part of the answer —
                    // an orientation only reachable outside them should count as unmatched).
                    if (movable[ji].HasLimits) next = Mathf.Clamp(next, lower[ji], upper[ji]);
                    else next = Mathf.Repeat(next + 180f, 360f) - 180f;
                    q[ji] = next;
                }
            }
            return false;
        }

        /// <summary>
        /// Conservative reachability test against the (dense, hole-filled) radial envelopes. The
        /// voxel map must NOT be used for this: it is a sparse Monte-Carlo scatter, so an empty
        /// voxel only means "no sample landed here" — at high map resolutions almost every voxel
        /// is empty and a voxel-based mask would skip the entire scan.
        /// </summary>
        private bool EnvelopeReachable(Vector3 local)
        {
            if (envelopeRadii == null || envelopeRadii.Length == 0) return true;
            Vector3 d = local - bubbleCenter;
            float radius = d.magnitude;
            float margin = Mathf.Max(0.02f * maxReach, 1e-3f);
            if (radius <= margin) return true;

            float theta = Mathf.Acos(Mathf.Clamp(d.y / radius, -1f, 1f));
            float phi = Mathf.Atan2(d.z, d.x);
            int lat = Mathf.Clamp(Mathf.RoundToInt(theta / Mathf.PI * radiiLatSegments), 0, radiiLatSegments);
            int lon = Mathf.RoundToInt((phi + Mathf.PI) / (2f * Mathf.PI) * radiiLonSegments) % radiiLonSegments;
            int index = lat * radiiLonSegments + lon;

            float inner = minEnvelopeRadii != null && minEnvelopeRadii.Length == envelopeRadii.Length
                ? minEnvelopeRadii[index] : 0f;
            return radius <= envelopeRadii[index] + margin && radius >= inner - margin;
        }

        // ---------------------------------------------------------------- Envelope grids

        /// <summary>
        /// Direction bins no sample touched hold <paramref name="emptyMarker"/>, which would spike
        /// the mesh. Grow filled bins outward into empty ones (average of filled neighbors).
        /// </summary>
        private float[] FillHoles(float[] radii, float emptyMarker)
        {
            int rows = radiiLatSegments + 1, cols = radiiLonSegments;
            for (int iteration = 0; iteration < rows + cols; iteration++)
            {
                bool anyEmpty = false;
                var next = (float[])radii.Clone();
                for (int lat = 0; lat < rows; lat++)
                for (int lon = 0; lon < cols; lon++)
                {
                    int index = lat * cols + lon;
                    if (radii[index] != emptyMarker) continue;

                    float sum = 0f; int filled = 0;
                    foreach (var (nl, nc) in Neighbors(lat, lon, rows, cols))
                    {
                        float r = radii[nl * cols + nc];
                        if (r != emptyMarker) { sum += r; filled++; }
                    }
                    if (filled > 0) next[index] = sum / filled;
                    else anyEmpty = true;
                }
                radii = next;
                if (!anyEmpty) break;
            }
            return radii;
        }

        private float[] Smooth(float[] radii)
        {
            int rows = radiiLatSegments + 1, cols = radiiLonSegments;
            var next = new float[radii.Length];
            for (int lat = 0; lat < rows; lat++)
            for (int lon = 0; lon < cols; lon++)
            {
                float sum = radii[lat * cols + lon];
                int count = 1;
                foreach (var (nl, nc) in Neighbors(lat, lon, rows, cols))
                {
                    sum += radii[nl * cols + nc];
                    count++;
                }
                next[lat * cols + lon] = sum / count;
            }
            return next;
        }

        private static IEnumerable<(int lat, int lon)> Neighbors(int lat, int lon, int rows, int cols)
        {
            if (lat > 0) yield return (lat - 1, lon);
            if (lat < rows - 1) yield return (lat + 1, lon);
            yield return (lat, (lon + 1) % cols);
            yield return (lat, (lon - 1 + cols) % cols);
        }

        // ---------------------------------------------------------------- Meshes

        public Mesh BuildEnvelopeMesh() => BuildRadialMesh(envelopeRadii, "Reach Envelope");
        public Mesh BuildMinEnvelopeMesh() => BuildRadialMesh(minEnvelopeRadii, "Reach Envelope (Inner)");

        /// <summary>Radial mesh in base-link local space (UV-sphere topology, radius per direction bin).</summary>
        private Mesh BuildRadialMesh(float[] radii, string meshName)
        {
            if (radii == null || radii.Length == 0) return null;

            int rows = radiiLatSegments + 1;
            int cols = radiiLonSegments + 1; // +1: duplicated seam column
            var vertices = new Vector3[rows * cols];

            for (int lat = 0; lat < rows; lat++)
            {
                float theta = lat / (float)radiiLatSegments * Mathf.PI;
                float sinT = Mathf.Sin(theta), cosT = Mathf.Cos(theta);
                for (int lon = 0; lon < cols; lon++)
                {
                    float phi = lon / (float)radiiLonSegments * 2f * Mathf.PI - Mathf.PI;
                    float radius = radii[lat * radiiLonSegments + lon % radiiLonSegments];
                    var direction = new Vector3(sinT * Mathf.Cos(phi), cosT, sinT * Mathf.Sin(phi));
                    vertices[lat * cols + lon] = bubbleCenter + direction * radius;
                }
            }

            var triangles = new List<int>(radiiLatSegments * radiiLonSegments * 6);
            for (int lat = 0; lat < radiiLatSegments; lat++)
            for (int lon = 0; lon < radiiLonSegments; lon++)
            {
                int a = lat * cols + lon;
                int b = a + 1;
                int c = a + cols;
                int d = c + 1;
                triangles.AddRange(new[] { a, b, c, b, d, c });
            }

            var mesh = new Mesh { name = meshName };
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        public void CreateMeshObject()
        {
            if (!HasResult)
            {
                Debug.LogWarning("[reach] No computed envelope — run Compute Reach Bubble first.", this);
                return;
            }
            var root = FrameLink();
            if (root == null) return;

            BakeMeshChild(root.transform, "Reach Bubble", BuildEnvelopeMesh(), envelopeColor);
            BakeMeshChild(root.transform, "Reach Bubble (Inner)", BuildMinEnvelopeMesh(), minEnvelopeColor);
        }

        private static void BakeMeshChild(Transform parent, string childName, Mesh mesh, Color color)
        {
            if (mesh == null) return;

            var existing = parent.Find(childName);
            var child = existing != null ? existing.gameObject : new GameObject(childName);
            child.transform.SetParent(parent, false);
            child.transform.localPosition = Vector3.zero;
            child.transform.localRotation = Quaternion.identity;

            var filter = child.GetComponent<MeshFilter>();
            if (filter == null) filter = child.AddComponent<MeshFilter>();
            filter.sharedMesh = mesh;

            var renderer = child.GetComponent<MeshRenderer>();
            if (renderer == null) renderer = child.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = CreateTransparentMaterial(color);
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        private static Material CreateTransparentMaterial(Color color)
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Standard");
            var material = new Material(shader) { name = "Reach Bubble" };
            material.color = color;
            MakeTransparent(material);
            return material;
        }

        // URP transparent surface setup (the inspector does the same when flipping Surface Type).
        private static void MakeTransparent(Material material)
        {
            if (material.HasProperty("_Surface")) material.SetFloat("_Surface", 1f);
            material.SetOverrideTag("RenderType", "Transparent");
            if (material.HasProperty("_SrcBlend")) material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            if (material.HasProperty("_DstBlend")) material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            if (material.HasProperty("_ZWrite")) material.SetInt("_ZWrite", 0);
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
        }

        // ---------------------------------------------------------------- Manipulability slice

        // Editor-only concept: in builds nothing is ever "selected", so the option hides everything.
        private bool VisualizationVisible()
        {
            if (!onlyWhenSelected) return true;
#if UNITY_EDITOR
            foreach (var selected in UnityEditor.Selection.transforms)
                if (selected == transform || selected.IsChildOf(transform))
                    return true;
#endif
            return false;
        }

        private void Update()
        {
            // A finished orientation scan can be shown even when the volumetric map is absent
            // (large maps are session-only, but the scan may have just been made).
            if (!showManipulabilitySlice || !(HasManipulabilityMap || hasScan) || !VisualizationVisible())
            {
                DestroySlice();
                return;
            }
            var frame = FrameLink();
            if (frame == null || (toolPointTransform == null && ToolLink() == null)) return;

            // Tool-point height in the base frame, clamped into the map cube. A finished
            // orientation scan is a snapshot at one height, so that mode pins the plane there.
            float y;
            if (sliceMode == SliceMode.OrientationScan && hasScan)
            {
                y = scanYLocal;
            }
            else
            {
                Vector3 toolLocal = frame.transform.InverseTransformPoint(ToolPointWorld());
                y = Mathf.Clamp(toolLocal.y, bubbleCenter.y - mapExtent, bubbleCenter.y + mapExtent);
            }

            EnsureSliceObject(frame);

            float cell = 2f * mapExtent / manipRes;
            if (float.IsNaN(sliceY) || Mathf.Abs(y - sliceY) > cell * 0.25f || sliceMode != lastSliceMode)
            {
                sliceY = y;
                lastSliceMode = sliceMode;
                RebuildSliceTexture();
            }
            sliceObject.transform.localPosition = new Vector3(bubbleCenter.x, sliceY, bubbleCenter.z);
        }

        private void OnDisable()
        {
#if UNITY_EDITOR
            UnityEditor.Selection.selectionChanged -= QueueEditorUpdate;
#endif
            DestroySlice();
        }

#if UNITY_EDITOR
        private void OnEnable()
        {
            // Edit-mode Update() only runs when the player loop is pumped; without this the slice
            // would linger (or fail to appear) after a selection change until some other repaint.
            UnityEditor.Selection.selectionChanged += QueueEditorUpdate;
        }

        private static void QueueEditorUpdate() => UnityEditor.EditorApplication.QueuePlayerLoopUpdate();
#endif

        private void EnsureSliceObject(UrdfLink frame)
        {
            if (sliceObject != null) return;

            // Reclaim a survivor from a domain reload rather than duplicating it. Never saved:
            // recreated from the serialized map on the next Update after any load.
            var existing = frame.transform.Find(SliceObjectName);
            sliceObject = existing != null ? existing.gameObject : new GameObject(SliceObjectName);
            sliceObject.hideFlags = HideFlags.HideAndDontSave;
            sliceObject.transform.SetParent(frame.transform, false);
            sliceObject.transform.localRotation = Quaternion.identity;

            var filter = sliceObject.GetComponent<MeshFilter>();
            if (filter == null) filter = sliceObject.AddComponent<MeshFilter>();
            DestroyEditorSafe(filter.sharedMesh); // reclaimed object may carry pre-reload leftovers
            filter.sharedMesh = BuildSliceQuad();

            var renderer = sliceObject.GetComponent<MeshRenderer>();
            if (renderer == null) renderer = sliceObject.AddComponent<MeshRenderer>();
            if (renderer.sharedMaterial != null)
            {
                DestroyEditorSafe(renderer.sharedMaterial.mainTexture);
                DestroyEditorSafe(renderer.sharedMaterial);
            }
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

            sliceTexture = new Texture2D(Mathf.Max(manipRes, 4), Mathf.Max(manipRes, 4), TextureFormat.RGBA32, false)
            {
                name = "Manipulability Slice",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                hideFlags = HideFlags.HideAndDontSave,
            };
            sliceY = float.NaN;

            var shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null) shader = Shader.Find("Unlit/Transparent");
            var material = new Material(shader) { name = "Manipulability Slice", hideFlags = HideFlags.HideAndDontSave };
            MakeTransparent(material);
            material.mainTexture = sliceTexture;
            if (material.HasProperty("_BaseMap")) material.SetTexture("_BaseMap", sliceTexture);
            renderer.sharedMaterial = material;
        }

        // Double-sided XZ quad spanning the map cube, centered on the slice object's origin.
        private Mesh BuildSliceQuad()
        {
            float e = mapExtent;
            var mesh = new Mesh { name = "Manipulability Slice", hideFlags = HideFlags.HideAndDontSave };
            mesh.SetVertices(new[]
            {
                new Vector3(-e, 0f, -e), new Vector3(e, 0f, -e),
                new Vector3(-e, 0f,  e), new Vector3(e, 0f,  e),
            });
            mesh.SetUVs(0, new[] { new Vector2(0, 0), new Vector2(1, 0), new Vector2(0, 1), new Vector2(1, 1) });
            mesh.SetTriangles(new[] { 0, 2, 1, 1, 2, 3, /* backside */ 0, 1, 2, 1, 3, 2 }, 0);
            mesh.RecalculateBounds();
            return mesh;
        }

        private void RebuildSliceTexture()
        {
            if (sliceTexture == null) return;

            if (sliceMode == SliceMode.OrientationScan && hasScan)
            {
                RebuildScanTexture();
                return;
            }
            if (!HasManipulabilityMap) return; // scan-only session: nothing volumetric to draw
            if (sliceTexture.width != manipRes) sliceTexture.Reinitialize(manipRes, manipRes);

            float cell = 2f * mapExtent / manipRes;
            int gy = Mathf.Clamp((int)((sliceY - (bubbleCenter.y - mapExtent)) / cell), 0, manipRes - 1);

            // Directional data exists only on maps baked since the vertical measure was added.
            bool hasVertical = vertManipMap != null && vertManipMap.Length == manipMap.Length && vertManipPeak > 0f;
            var mode = hasVertical ? sliceMode : SliceMode.Manipulability;

            var pixels = new Color32[manipRes * manipRes];
            byte alpha = (byte)(sliceOpacity * 255f);
            var singularBlue = new Color(0.15f, 0.3f, 1f);
            for (int gz = 0; gz < manipRes; gz++)
            for (int gx = 0; gx < manipRes; gx++)
            {
                int voxel = (gy * manipRes + gz) * manipRes + gx;
                byte v = manipMap[voxel];
                if (v == 0)
                {
                    pixels[gz * manipRes + gx] = new Color32(0, 0, 0, 0); // unreachable: transparent
                    continue;
                }

                // sqrt for contrast near zero; vertByte 0 on a reached voxel = exactly singular in Y.
                float tW = Mathf.Sqrt((v - 1) / 254f);
                float tV = 0f;
                if (hasVertical)
                {
                    byte vertByte = vertManipMap[voxel];
                    tV = vertByte == 0 ? 0f : Mathf.Sqrt((vertByte - 1) / 254f);
                }

                Color c;
                switch (mode)
                {
                    case SliceMode.VerticalManipulability:
                        c = Color.HSVToRGB(tV * 0.333f, 1f, 1f);
                        break;
                    case SliceMode.Combined:
                        // Yoshikawa hue, overridden by blue as the vertical capability collapses
                        // (fully blue at 0, no tint above ~25% of the vertical peak).
                        c = Color.HSVToRGB(tW * 0.333f, 1f, 1f);
                        c = Color.Lerp(c, singularBlue, Mathf.Clamp01(1f - tV * 2f));
                        break;
                    default:
                        c = Color.HSVToRGB(tW * 0.333f, 1f, 1f);
                        break;
                }
                pixels[gz * manipRes + gx] = new Color32((byte)(c.r * 255), (byte)(c.g * 255), (byte)(c.b * 255), alpha);
            }
            sliceTexture.SetPixels32(pixels);
            sliceTexture.Apply(false);
        }

        // Texture for the orientation-conditioned scan: red→green by full-pose w, overridden by
        // blue where the wrist axes approach collinear (the joint-4 flip locus at this orientation).
        private void RebuildScanTexture()
        {
            if (sliceTexture.width != scanRes) sliceTexture.Reinitialize(scanRes, scanRes);

            var pixels = new Color32[scanRes * scanRes];
            byte alpha = (byte)(sliceOpacity * 255f);
            var singularBlue = new Color(0.15f, 0.3f, 1f);
            var infeasibleGray = new Color32(60, 60, 60, (byte)(alpha / 2));
            float sinWarn = Mathf.Sin(wristWarnAngleDeg * Mathf.Deg2Rad);

            for (int cell = 0; cell < pixels.Length; cell++)
            {
                float wv = scanW[cell];
                if (wv <= -1.5f)
                {
                    pixels[cell] = infeasibleGray; // position reachable, orientation not — also useful to see
                }
                else if (wv < 0f)
                {
                    pixels[cell] = new Color32(0, 0, 0, 0); // out of reach entirely
                }
                else
                {
                    float t = scanWMax > 0f ? Mathf.Sqrt(wv / scanWMax) : 0f;
                    Color c = Color.HSVToRGB(t * 0.333f, 1f, 1f);
                    // Full blue at wristSin = 0, fading out at the warn threshold.
                    float blue = sinWarn > 0f ? Mathf.Clamp01(1f - scanWristSin[cell] / sinWarn) : 0f;
                    c = Color.Lerp(c, singularBlue, blue);
                    pixels[cell] = new Color32((byte)(c.r * 255), (byte)(c.g * 255), (byte)(c.b * 255), alpha);
                }
            }
            sliceTexture.SetPixels32(pixels);
            sliceTexture.Apply(false);
        }

        private void DestroySlice()
        {
            if (sliceObject == null) return;
            var renderer = sliceObject.GetComponent<MeshRenderer>();
            var filter = sliceObject.GetComponent<MeshFilter>();
            DestroyEditorSafe(renderer != null ? renderer.sharedMaterial : null);
            DestroyEditorSafe(filter != null ? filter.sharedMesh : null);
            DestroyEditorSafe(sliceTexture);
            DestroyEditorSafe(sliceObject);
            sliceObject = null;
            sliceTexture = null;
            sliceY = float.NaN;
        }

        private static void DestroyEditorSafe(Object obj)
        {
            if (obj == null) return;
            if (Application.isPlaying) Destroy(obj);
            else DestroyImmediate(obj);
        }

        // ---------------------------------------------------------------- Gizmos

        private UrdfLink FrameLink()
        {
            if (frameLink == null)
                frameLink = GetComponentsInChildren<UrdfLink>().FirstOrDefault(l => l.joint == null);
            return frameLink;
        }

        private UrdfLink ToolLink()
        {
            if (toolLink == null && !string.IsNullOrEmpty(computedForLink))
                toolLink = GetComponentsInChildren<UrdfLink>().FirstOrDefault(l => l.name == computedForLink);
            return toolLink;
        }

        /// <summary>The live world-space tool point the slice tracks (and the marker gizmo shows).</summary>
        private Vector3 ToolPointWorld()
        {
            if (toolPointTransform != null) return toolPointTransform.position;
            var tool = ToolLink();
            return tool != null ? tool.transform.TransformPoint(toolPointOffset) : transform.position;
        }

        private void OnDrawGizmos()
        {
            if (!enabled || !HasResult || !VisualizationVisible()) return;
            var root = FrameLink();
            if (root == null) return;

            Gizmos.matrix = root.transform.localToWorldMatrix;

            if (drawEnvelope)
            {
                if (envelopeMesh == null) envelopeMesh = BuildEnvelopeMesh();
                DrawEnvelopeGizmo(envelopeMesh, envelopeColor);
            }
            if (drawMinEnvelope)
            {
                if (minEnvelopeMesh == null) minEnvelopeMesh = BuildMinEnvelopeMesh();
                DrawEnvelopeGizmo(minEnvelopeMesh, minEnvelopeColor);
            }
            if (drawMaxSphere)
            {
                Gizmos.color = maxSphereColor;
                Gizmos.DrawWireSphere(bubbleCenter, maxReach);
            }
            if (drawMinSphere && minReach > 0f)
            {
                Gizmos.color = minSphereColor;
                Gizmos.DrawWireSphere(bubbleCenter, minReach);
            }

            // Tool-point marker: the exact point the slice height tracks, with a line from the end
            // link's origin (shows where the tool offset actually points) and a drop line to the plane.
            if (showManipulabilitySlice && HasManipulabilityMap)
            {
                Vector3 tcp = root.transform.InverseTransformPoint(ToolPointWorld());
                float markerSize = Mathf.Max(0.005f, maxReach * 0.02f);
                Gizmos.color = Color.white;
                var tool = ToolLink();
                if (tool != null)
                    Gizmos.DrawLine(root.transform.InverseTransformPoint(tool.transform.position), tcp);
                Gizmos.DrawWireSphere(tcp, markerSize);
                if (!float.IsNaN(sliceY))
                {
                    Gizmos.color = new Color(1f, 1f, 1f, 0.35f);
                    Gizmos.DrawLine(tcp, new Vector3(tcp.x, sliceY, tcp.z));
                }
            }

            // Live singularity gauge at the CURRENT configuration: blue ring + label when the outer
            // wrist axes approach collinear (the moment joint 4 would have to flip to hold orientation).
            if (showSingularityGauge)
            {
                var q = CurrentJointValues();
                if (q != null && EvaluateConfiguration(q, out Vector3 tcpNow, out float w6, out float wristSin))
                {
                    float wristDeg = Mathf.Asin(Mathf.Clamp01(wristSin)) * Mathf.Rad2Deg;
                    bool near = wristDeg < wristWarnAngleDeg;
                    Gizmos.color = near ? new Color(0.15f, 0.3f, 1f) : new Color(0f, 1f, 0.4f, 0.6f);
                    Gizmos.DrawWireSphere(tcpNow, Mathf.Max(0.01f, maxReach * (near ? 0.045f : 0.03f)));
#if UNITY_EDITOR
                    UnityEditor.Handles.Label(root.transform.TransformPoint(tcpNow),
                        $"  wrist {wristDeg:F1}°{(near ? "  ⚠ SINGULAR" : "")}   w6 {w6:F4}");
#endif
                }
            }
        }

        private static void DrawEnvelopeGizmo(Mesh mesh, Color color)
        {
            if (mesh == null) return;
            Gizmos.color = color;
            Gizmos.DrawMesh(mesh);
            Gizmos.color = new Color(color.r, color.g, color.b, Mathf.Min(1f, color.a * 2f));
            Gizmos.DrawWireMesh(mesh);
        }
    }
}
