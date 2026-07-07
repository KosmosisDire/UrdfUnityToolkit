using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
[RequireComponent(typeof(UrdfJointController))]
[RequireComponent(typeof(UrdfIKSolver))]
[ExecuteAlways]
public class UrdfIKController : MonoBehaviour
{
    public enum ConstraintFrame { World, Local }
    public enum ConstraintType { Cartesian, Joint }

    // A weighted IK constraint: a cartesian target on a link, or a target value for a single joint.
    [System.Serializable]
    public class IKConstraint
    {
        public ConstraintType type = ConstraintType.Cartesian;

        // Cartesian
        public Transform target;            // authority when set; otherwise the stored pose below
        public Vector3 position;
        public Quaternion rotation = Quaternion.identity;
        public string link = "";
        public ConstraintFrame frame = ConstraintFrame.Local;
        public bool posX = true, posY = true, posZ = true;
        public bool rotX = false, rotY = false, rotZ = false;

        // Joint space
        public string jointName = "";
        public float jointTarget = 0f;

        public float weight = 1f;

        public bool AnyPosition => posX || posY || posZ;
        public bool AnyRotation => rotX || rotY || rotZ;

        public Vector3 Position => target != null ? target.position : position;

        public Quaternion Rotation
        {
            get
            {
                if (target != null) return target.rotation;
                // Guard an uninitialized (zero) serialized quaternion.
                return rotation.x == 0 && rotation.y == 0 && rotation.z == 0 && rotation.w == 0 ? Quaternion.identity : rotation;
            }
        }

        public void SetPose(Vector3 pos, Quaternion rot)
        {
            if (target != null) { target.position = pos; target.rotation = rot; }
            else { position = pos; rotation = rot; }
        }
    }

    public List<IKConstraint> constraints = new List<IKConstraint>();

    private UrdfJointController jointController;
    private UrdfIKSolver ikSolver;
    private UrdfRobot robot;

    void OnEnable()
    {
        jointController = GetComponent<UrdfJointController>();
        ikSolver = GetComponent<UrdfIKSolver>();
        robot = GetComponent<UrdfRobot>();
    }

    void Update()
    {
        var tasks = BuildTasks();
        if (tasks.Count == 0) return;

        var result = ikSolver.SolveIK(tasks);
        if (result.jointPositions != null && result.jointPositions.Length > 0)
            ApplySolution(tasks, result.jointPositions);
    }

    // Map the configured constraints to solver tasks; skip empty/disabled ones.
    List<UrdfIKSolver.IKConstraintTask> BuildTasks()
    {
        var tasks = new List<UrdfIKSolver.IKConstraintTask>();
        if (constraints == null) return tasks;

        foreach (var c in constraints)
        {
            if (c == null || c.weight <= 0f) continue;

            if (c.type == ConstraintType.Joint)
            {
                if (string.IsNullOrEmpty(c.jointName)) continue;
                tasks.Add(new UrdfIKSolver.IKConstraintTask
                {
                    isJoint = true,
                    jointName = c.jointName,
                    jointTarget = c.jointTarget,
                    weight = c.weight,
                });
                continue;
            }

            if (string.IsNullOrEmpty(c.link) || (!c.AnyPosition && !c.AnyRotation)) continue;
            tasks.Add(new UrdfIKSolver.IKConstraintTask
            {
                linkName = c.link,
                position = c.Position,
                rotation = c.Rotation,
                px = c.posX, py = c.posY, pz = c.posZ,
                rx = c.rotX, ry = c.rotY, rz = c.rotZ,
                localFrame = c.frame == ConstraintFrame.Local,
                weight = c.weight,
            });
        }
        return tasks;
    }

    // Scatter the combined solution back into the controller array by full robot.joints index.
    void ApplySolution(List<UrdfIKSolver.IKConstraintTask> tasks, float[] solution)
    {
        if (robot == null || jointController == null || solution == null) return;

        var chain = ikSolver.GetCombinedChain(tasks);

        if (jointController.joints == null || jointController.joints.Length != robot.joints.Count)
        {
            var resized = new float[robot.joints.Count];
            if (jointController.joints != null)
                System.Array.Copy(jointController.joints, resized, Mathf.Min(jointController.joints.Length, resized.Length));
            jointController.joints = resized;
        }

        for (int i = 0; i < chain.Count && i < solution.Length; i++)
        {
            int index = robot.joints.IndexOf(chain[i]);
            if (index >= 0 && index < jointController.joints.Length)
                jointController.joints[index] = solution[i];
        }
    }
}
}
