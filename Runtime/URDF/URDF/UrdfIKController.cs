using UnityEngine;

namespace UrdfToolkit.Urdf
{
[RequireComponent(typeof(UrdfJointController))]
[RequireComponent(typeof(UrdfIKSolver))]
[ExecuteAlways]
public class UrdfIKController : MonoBehaviour
{
    private UrdfJointController jointController;
    private UrdfIKSolver ikSolver;
    private UrdfRobot robot;

    public Transform target;
    public string endEffectorLinkName = "tool_link";
    [SerializeField] private bool solveForRotation = true;

    void OnEnable()
    {
        jointController = GetComponent<UrdfJointController>();
        ikSolver = GetComponent<UrdfIKSolver>();
        robot = GetComponent<UrdfRobot>();
    }

    void Update()
    {
        if (target != null)
        {
            SolveIK();
        }
    }

    void SetTarget(Vector3 position, Quaternion rotation)
    {
        if (target == null)
        {
            target = new GameObject("IK Target").transform;
        }
        target.position = position;
        target.rotation = rotation;
    }

    void SolveIK()
    {
        UrdfIKSolver.IKResult result;

        if (solveForRotation)
        {
            result = ikSolver.SolveIK(endEffectorLinkName, target.position, target.rotation);
        }
        else
        {
            result = ikSolver.SolveIK(endEffectorLinkName, target.position);
        }

        // Always push the best solution found, even when not yet within tolerance. The solver
        // moves toward the target across frames (warm-starting from the previous solution), so
        // gating this would leave jointController.joints at 0 — the controller would then keep
        // yanking the arm back to 0 and fight the solve, and it could never converge.
        if (result.jointPositions != null && result.jointPositions.Length > 0)
        {
            ApplySolutionToController(result.jointPositions);
        }
    }

    /// <summary>
    /// Writes an IK solution into the joint controller's array. The solution is indexed by the
    /// solver's IK chain (only the movable joints to the end effector), while the controller applies
    /// its array by full <c>robot.joints</c> index — so we place each chain value at its joint's own
    /// index and leave every other joint untouched. (Assigning the chain-sized array directly would
    /// drive the wrong joints and force the rest to zero.)
    /// </summary>
    void ApplySolutionToController(float[] chainSolution)
    {
        if (robot == null || jointController == null || chainSolution == null) return;

        var chain = ikSolver.GetIKChain(endEffectorLinkName);

        // The controller applies joints[i] -> robot.joints[i], so its array must span every joint.
        if (jointController.joints == null || jointController.joints.Length != robot.joints.Count)
        {
            var resized = new float[robot.joints.Count];
            if (jointController.joints != null)
            {
                System.Array.Copy(jointController.joints, resized,
                    Mathf.Min(jointController.joints.Length, resized.Length));
            }
            jointController.joints = resized;
        }

        for (int i = 0; i < chain.Count && i < chainSolution.Length; i++)
        {
            int index = robot.joints.IndexOf(chain[i]);
            if (index >= 0 && index < jointController.joints.Length)
            {
                jointController.joints[index] = chainSolution[i];
            }
        }
    }
}
}
