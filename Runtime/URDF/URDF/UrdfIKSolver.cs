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
            
            // Track best solution
            if (posError < bestPosError || (posError == bestPosError && rotError < bestRotError))
            {
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
