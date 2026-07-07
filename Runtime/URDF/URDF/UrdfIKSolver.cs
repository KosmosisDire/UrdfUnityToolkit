using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UrdfToolkit.Urdf;

namespace UrdfToolkit.Urdf
{
[RequireComponent(typeof(UrdfRobot))]
[ExecuteAlways]
public class UrdfIKSolver : MonoBehaviour
{
    [Header("IK Settings")]
    [SerializeField] private int maxIterations = 50;
    [SerializeField] private float positionTolerance = 0.001f;
    [SerializeField] private float rotationTolerance = 0.01f;
    [SerializeField] private float jointTolerance = 0.01f; // joint-space constraint tolerance
    [SerializeField] private float dampingLambda = 0.1f; // Increased for stability
    [SerializeField] private float stepSize = 0.3f; // Reduced for smoother convergence
    
    [Header("Solver Options")]
    [SerializeField] private bool useJointLimits = true;
    [SerializeField] private bool useDamping = true;
    [SerializeField] private bool useSolutionCaching = true;
    [SerializeField] private float targetChangeTolerance = 0.0001f; // For detecting target changes
    [SerializeField] private float angleChangeTolerance = 0.1f; // Degrees
    
    [Header("Stability Settings")]
    [SerializeField] private bool useWarmStart = true; // Use last solution as starting point
    [SerializeField] private int refinementIterations = 5; // Fewer iterations when close to solution
    [SerializeField] private float refinementThreshold = 0.01f; // When to switch to refinement mode
    [SerializeField] private int restartAttempts = 8; // random seeds tried when the smooth solve is stuck
    [SerializeField] private int restartIterations = 200; // iterations per restart (must descend from far)
    
    private UrdfRobot robot;
    
    // Solution caching
    private Dictionary<string, float[]> lastSolutions = new Dictionary<string, float[]>();
    private Dictionary<string, (Vector3 pos, Quaternion? rot)> lastTargets = new Dictionary<string, (Vector3, Quaternion?)>();
    private Dictionary<string, float[]> savedStates = new Dictionary<string, float[]>(); // For state restoration
    
    public struct IKResult
    {
        public bool success;
        public float[] jointPositions;
        public int iterations;
        public float positionError;
        public float rotationError;

        public IKResult(bool success, float[] positions, int iterations, float posError, float rotError)
        {
            this.success = success;
            this.jointPositions = positions;
            this.iterations = iterations;
            this.positionError = posError;
            this.rotationError = rotError;
        }
    }

    // One weighted IK constraint: cartesian (axis-masked link pose) or joint-space (single joint value).
    public struct IKConstraintTask
    {
        public bool isJoint;
        // Cartesian
        public string linkName;
        public Vector3 position;
        public Quaternion rotation;
        public bool px, py, pz, rx, ry, rz; // active task axes
        public bool localFrame;             // mask axes in the link's frame vs world
        // Joint space
        public string jointName;
        public float jointTarget;
        public float weight;
    }
    
    void OnEnable()
    {
        robot = GetComponent<UrdfRobot>();
        ClearCache();
    }
    
    /// <summary>
    /// Clear all cached solutions
    /// </summary>
    public void ClearCache()
    {
        lastSolutions.Clear();
        lastTargets.Clear();
        savedStates.Clear();
    }
    
    /// <summary>
    /// The ordered list of joints the solver actually drives for a given end effector: the
    /// controllable (non-fixed) joints in the kinematic chain, restricted to single-DOF
    /// revolute/continuous and prismatic joints. <see cref="IKResult.jointPositions"/> is indexed to
    /// match this list, so use it (not <c>robot.joints</c>) to map a solution back onto joints.
    /// </summary>
    public List<UrdfJoint> GetIKChain(string endEffectorName)
    {
        if (robot == null) robot = GetComponent<UrdfRobot>();
        if (robot == null) return new List<UrdfJoint>();

        return robot.GetControllableJointChain(endEffectorName)
            .Where(j => j is UrdfJointRevolute || j is UrdfJointPrismatic)
            .ToList();
    }

    // Union of all joints the tasks touch: cartesian chains plus any directly constrained joint.
    public List<UrdfJoint> GetCombinedChain(List<IKConstraintTask> tasks)
    {
        if (robot == null) robot = GetComponent<UrdfRobot>();
        var combined = new List<UrdfJoint>();
        var seen = new HashSet<UrdfJoint>();
        foreach (var t in tasks)
        {
            if (t.isJoint)
            {
                var j = ResolveJoint(t.jointName);
                if (j != null && seen.Add(j)) combined.Add(j);
            }
            else
            {
                foreach (var j in GetIKChain(t.linkName))
                    if (seen.Add(j)) combined.Add(j);
            }
        }
        return combined;
    }

    // Find a controllable joint by its URDF name (falling back to the GameObject name).
    UrdfJoint ResolveJoint(string name)
    {
        if (robot == null || string.IsNullOrEmpty(name)) return null;
        foreach (var j in robot.joints)
            if (j != null && (j.JointName == name || j.name == name)) return j;
        return null;
    }

    // Shared per-solve setup, reused across seeds.
    private struct SolveContext
    {
        public List<IKConstraintTask> tasks;
        public List<UrdfJoint>[] chains;   // cartesian chain per task (null for joint tasks)
        public int[] jointCol;             // joint-task column (-1 otherwise)
        public Dictionary<UrdfJoint, int> column;
        public int dof;
        public float[] lower, upper;
    }

    private struct SolveAttempt
    {
        public float[] q;
        public bool converged;
        public bool stalled;        // plateaued at a local minimum (not just unfinished this frame)
        public float score;         // tolerance-normalized residual (lower is better)
        public int iterations;
        public float posErr, rotErr;
        public IKResult Result() => new IKResult(converged, q, iterations, posErr, rotErr);
    }

    /// <summary>
    /// Multi-constraint IK: weighted stacked Jacobian damped least squares over the union of the
    /// constraints' joints. Cartesian tasks emit their active (optionally link-frame) axes; joint
    /// tasks emit one row pinning a joint to a value. Constraints may share joints; weights trade off.
    ///
    /// Solves from the current pose first (the smooth, nearest solution). If that can't reach the
    /// target (local minimum / joint lock), it retries from random seeds and takes the converged one
    /// closest to the current pose. If nothing converges it keeps the smooth best-effort.
    /// </summary>
    public IKResult SolveIK(List<IKConstraintTask> tasks)
    {
        if (robot == null) robot = GetComponent<UrdfRobot>();
        if (robot == null || tasks == null || tasks.Count == 0)
            return new IKResult(false, new float[0], 0, float.MaxValue, float.MaxValue);

        var combined = GetCombinedChain(tasks);
        int dof = combined.Count;
        if (dof == 0)
            return new IKResult(false, new float[0], 0, float.MaxValue, float.MaxValue);

        var column = new Dictionary<UrdfJoint, int>();
        for (int i = 0; i < dof; i++) column[combined[i]] = i;

        // Per-task data: cartesian chain, or the column of a joint task (-1 if unresolved).
        var chains = new List<UrdfJoint>[tasks.Count];
        var jointCol = new int[tasks.Count];
        for (int k = 0; k < tasks.Count; k++)
        {
            if (tasks[k].isJoint)
            {
                var j = ResolveJoint(tasks[k].jointName);
                jointCol[k] = j != null && column.TryGetValue(j, out var ci) ? ci : -1;
            }
            else
            {
                chains[k] = GetIKChain(tasks[k].linkName);
                jointCol[k] = -1;
            }
        }

        float[] current = combined.Select(j => j.GetPosition()).ToArray();
        var ctx = new SolveContext
        {
            tasks = tasks, chains = chains, jointCol = jointCol, column = column, dof = dof,
            lower = combined.Select(j => (float)j.LowerLimit).ToArray(),
            upper = combined.Select(j => (float)j.UpperLimit).ToArray(),
        };

        // Use the warm solve while it converges or is still progressing; only a stall is joint lock.
        var warm = RunFromSeed(ctx, current, maxIterations);
        if (warm.converged || !warm.stalled) return warm.Result();

        // Stalled at a local minimum: random restarts with a bigger budget so they descend from far.
        SolveAttempt closestConverged = default;
        float closestDist = float.MaxValue;
        SolveAttempt bestPartial = warm;
        for (int r = 0; r < restartAttempts; r++)
        {
            var res = RunFromSeed(ctx, RandomSeed(current, ctx.lower, ctx.upper), restartIterations);
            if (res.converged)
            {
                float dist = JointDistanceSq(res.q, current);
                if (dist < closestDist) { closestDist = dist; closestConverged = res; }
            }
            else if (res.score < bestPartial.score) bestPartial = res;
        }

        if (closestDist < float.MaxValue) return closestConverged.Result();   // a real solution: closest jump
        if (bestPartial.score < warm.score * 0.5f) return bestPartial.Result(); // escaped to a much better basin
        return warm.Result();                                                  // nothing better: stay (stable)
    }

    // One damped-least-squares descent from a given seed configuration.
    private SolveAttempt RunFromSeed(SolveContext ctx, float[] seed, int maxIters)
    {
        const int stallWindow = 5;      // iterations without meaningful gain that mean "stuck"
        const float improveEps = 0.01f; // min score drop (in tolerances) that counts as progress

        int dof = ctx.dof;
        float[] q = (float[])seed.Clone();
        float[] best = (float[])q.Clone();
        float bestScore = float.MaxValue, bestPos = float.MaxValue, bestRot = 0f;
        int lastImprove = 0;

        for (int iteration = 0; iteration < maxIters; iteration++)
        {
            var rows = new List<float[]>(); // weighted Jacobian rows over the combined dof
            var errs = new List<float>();   // matching weighted errors
            float maxPos = 0f, maxRot = 0f, maxJoint = 0f;

            for (int k = 0; k < ctx.tasks.Count; k++)
            {
                var t = ctx.tasks[k];
                if (t.weight <= 0f) continue;

                // Joint-space task: one row, identity at the joint's column.
                if (t.isJoint)
                {
                    int col = ctx.jointCol[k];
                    if (col < 0) continue;
                    float ej = t.jointTarget - q[col];
                    float[] jrow = new float[dof];
                    jrow[col] = t.weight;
                    rows.Add(jrow);
                    errs.Add(t.weight * ej);
                    maxJoint = Mathf.Max(maxJoint, Mathf.Abs(ej));
                    continue;
                }

                var chain = ctx.chains[k];
                if (chain == null || chain.Count == 0) continue;

                float[] qk = new float[chain.Count];
                for (int c = 0; c < chain.Count; c++) qk[c] = q[ctx.column[chain[c]]];

                ChainPose pose = EvaluateFK(t.linkName, chain, qk);

                Vector3 ePos = t.position - pose.eePosition;
                Quaternion rd = t.rotation * Quaternion.Inverse(pose.eeRotation);
                rd.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180) angle -= 360;
                Vector3 eRot = axis * (angle * Mathf.Deg2Rad);

                // Express error and Jacobian rows in the chosen frame (identity for world).
                Quaternion frameInv = t.localFrame ? Quaternion.Inverse(pose.eeRotation) : Quaternion.identity;
                Vector3 ePosF = frameInv * ePos;
                Vector3 eRotF = frameInv * eRot;

                float[,] Jk = ComputeJacobian(chain, pose.jointPos, pose.jointAxis, pose.eePosition, 6);
                Vector3[] linF = new Vector3[chain.Count];
                Vector3[] angF = new Vector3[chain.Count];
                for (int c = 0; c < chain.Count; c++)
                {
                    linF[c] = frameInv * new Vector3(Jk[0, c], Jk[1, c], Jk[2, c]);
                    angF[c] = frameInv * new Vector3(Jk[3, c], Jk[4, c], Jk[5, c]);
                }

                bool[] active = { t.px, t.py, t.pz, t.rx, t.ry, t.rz };
                for (int a = 0; a < 6; a++)
                {
                    if (!active[a]) continue;

                    float[] rowVec = new float[dof];
                    for (int c = 0; c < chain.Count; c++)
                        rowVec[ctx.column[chain[c]]] = t.weight * (a < 3 ? linF[c][a] : angF[c][a - 3]);

                    float e = a < 3 ? ePosF[a] : eRotF[a - 3];
                    rows.Add(rowVec);
                    errs.Add(t.weight * e);

                    if (a < 3) maxPos = Mathf.Max(maxPos, Mathf.Abs(e));
                    else maxRot = Mathf.Max(maxRot, Mathf.Abs(e));
                }
            }

            int taskDim = rows.Count;
            if (taskDim == 0) // nothing active to drive
                return new SolveAttempt { q = (float[])q.Clone(), converged = true, iterations = iteration };

            float[,] J = new float[taskDim, dof];
            float[] error = new float[taskDim];
            for (int r = 0; r < taskDim; r++)
            {
                error[r] = errs[r];
                for (int c = 0; c < dof; c++) J[r, c] = rows[r][c];
            }

            // Keep the best pose by tolerance-normalized residual (see SolutionScore).
            float score = maxPos / Mathf.Max(positionTolerance, 1e-6f)
                        + maxRot / Mathf.Max(rotationTolerance, 1e-6f)
                        + maxJoint / Mathf.Max(jointTolerance, 1e-6f);
            if (score < bestScore)
            {
                if (score < bestScore - improveEps) lastImprove = iteration;
                bestScore = score; bestPos = maxPos; bestRot = maxRot; best = (float[])q.Clone();
            }

            if (maxPos < positionTolerance && maxRot < rotationTolerance && maxJoint < jointTolerance)
                return new SolveAttempt { q = (float[])q.Clone(), converged = true, iterations = iteration, posErr = maxPos, rotErr = maxRot };

            float damping = dampingLambda;
            if (EstimateConditionNumber(J, taskDim, dof) > 100) damping *= 10f;

            float[] dq = SolvePseudoInverse(J, error, dof, taskDim, damping);
            for (int i = 0; i < dof; i++)
            {
                q[i] += dq[i] * stepSize;
                if (useJointLimits) q[i] = Mathf.Clamp(q[i], ctx.lower[i], ctx.upper[i]);
            }
        }

        bool stalled = (maxIters - 1 - lastImprove) >= stallWindow;
        return new SolveAttempt { q = best, converged = false, stalled = stalled, score = bestScore, iterations = maxIters, posErr = bestPos, rotErr = bestRot };
    }

    // Restart seed: keep most joints near the current pose, fully randomize a few (escapes joint locks).
    private float[] RandomSeed(float[] current, float[] lower, float[] upper)
    {
        float[] seed = new float[current.Length];
        for (int i = 0; i < seed.Length; i++)
        {
            bool finite = !float.IsInfinity(lower[i]) && !float.IsInfinity(upper[i]) && upper[i] > lower[i];
            float lo = finite ? lower[i] : current[i] - 180f;
            float hi = finite ? upper[i] : current[i] + 180f;
            seed[i] = UnityEngine.Random.value < 0.3f
                ? UnityEngine.Random.Range(lo, hi)
                : Mathf.Clamp(current[i] + 0.1f * (hi - lo) * UnityEngine.Random.Range(-1f, 1f), lo, hi);
        }
        return seed;
    }

    // Squared joint-space distance, used to pick the restart closest to the current pose.
    private static float JointDistanceSq(float[] a, float[] b)
    {
        float s = 0f;
        for (int i = 0; i < a.Length && i < b.Length; i++) { float d = a[i] - b[i]; s += d * d; }
        return s;
    }

    /// <summary>
    /// Solve IK for position only
    /// </summary>
    public IKResult SolveIK(string endEffectorName, Vector3 targetPosition)
    {
        return SolveIK(endEffectorName, targetPosition, null);
    }
    
    /// <summary>
    /// Solve IK for position and rotation
    /// </summary>
    public IKResult SolveIK(string endEffectorName, Vector3 targetPosition, Quaternion targetRotation)
    {
        return SolveIK(endEffectorName, targetPosition, (Quaternion?)targetRotation);
    }
    
    /// <summary>
    /// Check if target has changed significantly
    /// </summary>
    private bool HasTargetChanged(string endEffectorName, Vector3 targetPosition, Quaternion? targetRotation)
    {
        if (!lastTargets.ContainsKey(endEffectorName))
            return true;
        
        var last = lastTargets[endEffectorName];
        
        // Check position change
        if ((last.pos - targetPosition).magnitude > targetChangeTolerance)
            return true;
        
        // Check rotation change
        if (targetRotation.HasValue && last.rot.HasValue)
        {
            float angleDiff = Quaternion.Angle(targetRotation.Value, last.rot.Value);
            if (angleDiff > angleChangeTolerance)
                return true;
        }
        else if (targetRotation.HasValue != last.rot.HasValue)
        {
            return true;
        }
        
        return false;
    }
    
    /// <summary>Forward-kinematics result: end-effector pose plus each solve joint's world pivot/axis.</summary>
    private struct ChainPose
    {
        public Vector3 eePosition;
        public Quaternion eeRotation;
        public Vector3[] jointPos;   // world pivot of each solve-chain joint
        public Vector3[] jointAxis;  // world axis of each solve-chain joint
    }

    /// <summary>
    /// Evaluates forward kinematics for a candidate joint configuration ENTIRELY IN MATH — it never
    /// moves the live robot, so it is valid whether the robot is kinematically posed or being driven
    /// by physics (ArticulationBody). Walks the full joint path (including fixed joints, for their
    /// constant offsets), applying each joint's <see cref="UrdfJoint.LocalMotionMatrix"/>. The
    /// <paramref name="q"/> values are indexed by <paramref name="solveChain"/> (the non-fixed joints).
    /// </summary>
    private ChainPose EvaluateFK(string endEffectorName, List<UrdfJoint> solveChain, float[] q)
    {
        var fullPath = robot.GetJointChain(endEffectorName);

        var pose = new ChainPose
        {
            jointPos = new Vector3[solveChain.Count],
            jointAxis = new Vector3[solveChain.Count],
        };

        // Base frame: the world transform of the path's first joint's parent link, which sits above
        // the chain and stays fixed during the solve (the robot's root / immovable base).
        Matrix4x4 world;
        if (fullPath.Count > 0 && fullPath[0].parentLink != null)
            world = fullPath[0].parentLink.transform.localToWorldMatrix;
        else if (robot.RootLink != null)
            world = robot.RootLink.transform.localToWorldMatrix;
        else
            world = Matrix4x4.identity;

        int qIndex = 0;
        foreach (var joint in fullPath)
        {
            bool controllable = !joint.IsFixed;
            float value = (controllable && qIndex < q.Length) ? q[qIndex] : 0f;

            world *= joint.LocalMotionMatrix(value);

            if (controllable)
            {
                if (qIndex < pose.jointPos.Length)
                {
                    pose.jointPos[qIndex] = world.GetColumn(3);                    // pivot = link origin
                    pose.jointAxis[qIndex] = world.MultiplyVector(joint.LocalAxis).normalized;
                }
                qIndex++;
            }
        }

        pose.eePosition = world.GetColumn(3);
        pose.eeRotation = world.rotation;
        return pose;
    }

    /// <summary>
    /// Main IK solver using Jacobian pseudo-inverse method
    /// </summary>
    private IKResult SolveIK(string endEffectorName, Vector3 targetPosition, Quaternion? targetRotation)
    {
        // Get the movable joints the solver drives (see GetIKChain). IKResult.jointPositions is
        // indexed to match this list, so callers must use GetIKChain to map results back to joints.
        var jointChain = GetIKChain(endEffectorName);
        if (jointChain.Count == 0)
        {
            Debug.LogWarning($"No movable (revolute/prismatic) joints in chain to {endEffectorName}");
            return new IKResult(false, new float[0], 0, float.MaxValue, float.MaxValue);
        }
        
        var endEffectorLink = robot.GetLink(endEffectorName);
        if (endEffectorLink == null)
        {
            Debug.LogError($"End effector link {endEffectorName} not found");
            return new IKResult(false, new float[0], 0, float.MaxValue, float.MaxValue);
        }
        
        // Check if we can use cached solution
        if (useSolutionCaching && !HasTargetChanged(endEffectorName, targetPosition, targetRotation))
        {
            if (lastSolutions.ContainsKey(endEffectorName))
            {
                // Verify the cached solution is still valid (analytical FK — never moves the robot).
                float[] cachedSolution = lastSolutions[endEffectorName];
                ChainPose cachedPose = EvaluateFK(endEffectorName, jointChain, cachedSolution);

                Vector3 currPos = cachedPose.eePosition;
                float posErr = (targetPosition - currPos).magnitude;

                float rotErr = 0f;
                if (targetRotation.HasValue)
                {
                    rotErr = Quaternion.Angle(targetRotation.Value, cachedPose.eeRotation) * Mathf.Deg2Rad;
                }
                
                if (posErr < positionTolerance && (!targetRotation.HasValue || rotErr < rotationTolerance))
                {
                    return new IKResult(true, cachedSolution, 0, posErr, rotErr);
                }
            }
        }
        
        // Save current state for restoration
        float[] originalPositions = robot.GetJointPositions(jointChain);
        
        // Determine starting positions
        float[] startPositions;
        if (useWarmStart && lastSolutions.ContainsKey(endEffectorName) && 
            !HasTargetChanged(endEffectorName, targetPosition, targetRotation))
        {
            // Use last solution as starting point
            startPositions = (float[])lastSolutions[endEffectorName].Clone();
        }
        else
        {
            // Use current positions
            startPositions = (float[])originalPositions.Clone();
        }
        
        // Check if we're already at the target (early exit). FK is analytical — see EvaluateFK.
        ChainPose pose = EvaluateFK(endEffectorName, jointChain, startPositions);
        Vector3 currentPos = pose.eePosition;
        Quaternion currentRot = pose.eeRotation;
        
        float posError = (targetPosition - currentPos).magnitude;
        float rotError = 0f;
        
        if (targetRotation.HasValue)
        {
            Quaternion rotDiff = targetRotation.Value * Quaternion.Inverse(currentRot);
            rotDiff.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180) angle -= 360;
            rotError = Mathf.Abs(angle * Mathf.Deg2Rad);
        }
        
        // Early exit if already converged
        if (posError < positionTolerance && (!targetRotation.HasValue || rotError < rotationTolerance))
        {
            // Update cache
            lastTargets[endEffectorName] = (targetPosition, targetRotation);
            lastSolutions[endEffectorName] = (float[])startPositions.Clone();
            
            return new IKResult(true, startPositions, 0, posError, rotError);
        }
        
        // Determine if we should use refinement mode (fewer iterations, smaller steps)
        bool useRefinement = posError < refinementThreshold && 
                            (!targetRotation.HasValue || rotError < refinementThreshold);
        int actualMaxIterations = useRefinement ? refinementIterations : maxIterations;
        float actualStepSize = useRefinement ? stepSize * 0.5f : stepSize;
        
        // Get joint limits if needed
        var (lowerLimits, upperLimits) = useJointLimits ? 
            robot.GetJointLimits(jointChain) : 
            (null, null);
        
        int dof = jointChain.Count;
        bool useRotation = targetRotation.HasValue;
        int taskDim = useRotation ? 6 : 3;
        
        // Working positions for iteration
        float[] resultPositions = (float[])startPositions.Clone();
        
        // Iterate until convergence or max iterations
        int iteration = 0;
        float bestPosError = posError;
        float bestRotError = rotError;
        float bestScore = SolutionScore(posError, rotError, useRotation);
        float[] bestSolution = (float[])resultPositions.Clone();
        
        for (iteration = 0; iteration < actualMaxIterations; iteration++)
        {
            // Evaluate forward kinematics analytically for the candidate configuration. This does NOT
            // touch the live transforms, so it works whether the robot is kinematic or physics-driven.
            pose = EvaluateFK(endEffectorName, jointChain, resultPositions);
            currentPos = pose.eePosition;
            currentRot = pose.eeRotation;
            
            // Calculate error
            Vector3 positionError = targetPosition - currentPos;
            posError = positionError.magnitude;
            
            Vector3 rotationError = Vector3.zero;
            if (useRotation)
            {
                Quaternion rotDiff = targetRotation.Value * Quaternion.Inverse(currentRot);
                rotDiff.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180) angle -= 360;
                rotationError = axis * (angle * Mathf.Deg2Rad);
                rotError = rotationError.magnitude;
            }
            
            // Track the best solution by a combined position+rotation score.
            float score = SolutionScore(posError, rotError, useRotation);
            if (score < bestScore)
            {
                bestScore = score;
                bestPosError = posError;
                bestRotError = rotError;
                bestSolution = (float[])resultPositions.Clone();
            }
            
            // Check convergence
            if (posError < positionTolerance && (!useRotation || rotError < rotationTolerance))
            {
                // Update cache
                lastTargets[endEffectorName] = (targetPosition, targetRotation);
                lastSolutions[endEffectorName] = (float[])resultPositions.Clone();
                
                return new IKResult(true, resultPositions, iteration, posError, rotError);
            }
            
            // Build Jacobian matrix from the analytical joint poses.
            float[,] J = ComputeJacobian(jointChain, pose.jointPos, pose.jointAxis, currentPos, taskDim);
            
            // Check for singularity (optional - add condition number check)
            float conditionNumber = EstimateConditionNumber(J, taskDim, dof);
            float actualDamping = dampingLambda;
            if (conditionNumber > 100) // Near singularity
            {
                actualDamping *= 10; // Increase damping
            }
            
            // Build error vector
            float[] errorVector = new float[taskDim];
            errorVector[0] = positionError.x;
            errorVector[1] = positionError.y;
            errorVector[2] = positionError.z;
            if (useRotation)
            {
                errorVector[3] = rotationError.x;
                errorVector[4] = rotationError.y;
                errorVector[5] = rotationError.z;
            }
            
            // Solve for joint velocities using pseudo-inverse
            float[] deltaQ = SolvePseudoInverse(J, errorVector, dof, taskDim, actualDamping);
            
            // Apply step size
            for (int i = 0; i < dof; i++)
            {
                deltaQ[i] *= actualStepSize;
            }
            
            // Update joint positions
            for (int i = 0; i < dof; i++)
            {
                resultPositions[i] += deltaQ[i];
                
                // Apply joint limits
                if (useJointLimits && lowerLimits != null && upperLimits != null)
                {
                    resultPositions[i] = Mathf.Clamp(resultPositions[i], lowerLimits[i], upperLimits[i]);
                }
            }
        }
        
        // Use best solution found
        resultPositions = bestSolution;
        
        // Update cache even for partial solutions if they're better than nothing
        if (bestPosError < positionTolerance * 10)
        {
            lastTargets[endEffectorName] = (targetPosition, targetRotation);
            lastSolutions[endEffectorName] = (float[])resultPositions.Clone();
        }
        
        // Max iterations reached
        return new IKResult(false, resultPositions, iteration, bestPosError, bestRotError);
    }
    
    private float SolutionScore(float posError, float rotError, bool useRotation)
    {
        float score = posError / Mathf.Max(positionTolerance, 1e-6f);
        if (useRotation)
            score += rotError / Mathf.Max(rotationTolerance, 1e-6f);
        return score;
    }

    /// <summary>
    /// Estimate condition number of Jacobian for singularity detection
    /// </summary>
    private float EstimateConditionNumber(float[,] J, int rows, int cols)
    {
        // Simple estimation using Frobenius norm
        float norm = 0;
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                norm += J[i, j] * J[i, j];
            }
        }
        return Mathf.Sqrt(norm);
    }
    
    /// <summary>
    /// Compute the Jacobian matrix for the current configuration
    /// </summary>
    private float[,] ComputeJacobian(List<UrdfJoint> jointChain, Vector3[] jointPositions, Vector3[] jointAxes, Vector3 endEffectorPos, int taskDim)
    {
        int dof = jointChain.Count;
        float[,] J = new float[taskDim, dof];
        
        for (int i = 0; i < dof; i++)
        {
            // World pivot and axis come from the analytical FK pass (no live-transform reads).
            Vector3 jointPos = jointPositions[i];
            Vector3 jointAxis = jointAxes[i];

            Vector3 linearVel;
            Vector3 angularVel;

            if (jointChain[i] is UrdfJointPrismatic)
            {
                // Prismatic: sliding along the axis is pure translation, no rotation.
                linearVel = jointAxis;
                angularVel = Vector3.zero;
            }
            else
            {
                // Revolute/continuous: rotation about the axis.
                // Linear contribution: axis × (end_effector - joint_position); angular: the axis.
                linearVel = Vector3.Cross(jointAxis, endEffectorPos - jointPos);
                angularVel = jointAxis;
            }

            J[0, i] = linearVel.x;
            J[1, i] = linearVel.y;
            J[2, i] = linearVel.z;

            if (taskDim == 6)
            {
                J[3, i] = angularVel.x;
                J[4, i] = angularVel.y;
                J[5, i] = angularVel.z;
            }
        }
        
        return J;
    }
    
    /// <summary>
    /// Solve using damped pseudo-inverse: J+ = JT(JJT + λ²I)^-1
    /// </summary>
    private float[] SolvePseudoInverse(float[,] J, float[] error, int dof, int taskDim, float damping = -1)
    {
        if (damping < 0) damping = dampingLambda;
        
        float[] result = new float[dof];
        
        if (useDamping)
        {
            // Damped least squares (Levenberg-Marquardt)
            // Compute JJT + λ²I
            float[,] JJT = new float[taskDim, taskDim];
            for (int i = 0; i < taskDim; i++)
            {
                for (int j = 0; j < taskDim; j++)
                {
                    float sum = 0;
                    for (int k = 0; k < dof; k++)
                    {
                        sum += J[i, k] * J[j, k];
                    }
                    JJT[i, j] = sum;
                    if (i == j) JJT[i, j] += damping * damping;
                }
            }
            
            // Solve (JJT + λ²I)y = error for y
            float[] y = SolveLinearSystem(JJT, error, taskDim);
            
            // Compute result = JT * y
            for (int i = 0; i < dof; i++)
            {
                float sum = 0;
                for (int j = 0; j < taskDim; j++)
                {
                    sum += J[j, i] * y[j];
                }
                result[i] = sum;
            }
        }
        else
        {
            // Simple transpose method (works for redundant systems)
            // result = JT * error
            for (int i = 0; i < dof; i++)
            {
                float sum = 0;
                for (int j = 0; j < taskDim; j++)
                {
                    sum += J[j, i] * error[j];
                }
                result[i] = sum;
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Simple Gaussian elimination for solving Ax = b
    /// </summary>
    private float[] SolveLinearSystem(float[,] A, float[] b, int n)
    {
        // Create augmented matrix
        float[,] aug = new float[n, n + 1];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                aug[i, j] = A[i, j];
            }
            aug[i, n] = b[i];
        }
        
        // Forward elimination
        for (int i = 0; i < n; i++)
        {
            // Partial pivoting
            int maxRow = i;
            for (int k = i + 1; k < n; k++)
            {
                if (Mathf.Abs(aug[k, i]) > Mathf.Abs(aug[maxRow, i]))
                {
                    maxRow = k;
                }
            }
            
            // Swap rows
            if (maxRow != i)
            {
                for (int k = i; k <= n; k++)
                {
                    float temp = aug[i, k];
                    aug[i, k] = aug[maxRow, k];
                    aug[maxRow, k] = temp;
                }
            }
            
            // Make diagonal 1
            float pivot = aug[i, i];
            if (Mathf.Abs(pivot) < 1e-10) continue;
            
            for (int j = i; j <= n; j++)
            {
                aug[i, j] /= pivot;
            }
            
            // Eliminate column
            for (int k = i + 1; k < n; k++)
            {
                float factor = aug[k, i];
                for (int j = i; j <= n; j++)
                {
                    aug[k, j] -= factor * aug[i, j];
                }
            }
        }
        
        // Back substitution
        float[] x = new float[n];
        for (int i = n - 1; i >= 0; i--)
        {
            x[i] = aug[i, n];
            for (int j = i + 1; j < n; j++)
            {
                x[i] -= aug[i, j] * x[j];
            }
        }
        
        return x;
    }
    
    /// <summary>
    /// Helper method to solve and apply IK in one call
    /// </summary>
    public bool SolveAndApplyIK(string endEffectorName, Vector3 targetPosition, Quaternion? targetRotation = null)
    {
        var result = SolveIK(endEffectorName, targetPosition, targetRotation);
        
        if (result.success || result.positionError < positionTolerance * 10) // Accept near-solutions
        {
            robot.SetJointPositions(GetIKChain(endEffectorName), result.jointPositions);
            return result.success;
        }
        
        return false;
    }
    
    /// <summary>
    /// Force invalidate cache for a specific end effector
    /// </summary>
    public void InvalidateCache(string endEffectorName)
    {
        if (lastSolutions.ContainsKey(endEffectorName))
            lastSolutions.Remove(endEffectorName);
        if (lastTargets.ContainsKey(endEffectorName))
            lastTargets.Remove(endEffectorName);
    }
    
    /// <summary>
    /// Get current IK chain info for debugging
    /// </summary>
    public void LogChainInfo(string endEffectorName)
    {
        var chain = robot.GetJointChain(endEffectorName);
        Debug.Log($"IK Chain to {endEffectorName}:");
        foreach (var joint in chain)
        {
            Debug.Log($"  {joint.name}: Type={joint.GetType().Name}, Pos={joint.GetPosition():F3}, " +
                     $"Limits=[{joint.LowerLimit:F2}, {joint.UpperLimit:F2}]");
        }
        
        // Log cache status
        if (lastSolutions.ContainsKey(endEffectorName))
        {
            Debug.Log($"  Cached solution available");
        }
        if (lastTargets.ContainsKey(endEffectorName))
        {
            var target = lastTargets[endEffectorName];
            Debug.Log($"  Last target: {target.pos}, Rot: {target.rot}");
        }
    }
}
}
