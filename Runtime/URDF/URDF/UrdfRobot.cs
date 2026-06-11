using System.Collections.Generic;
using System.Linq;
using UrdfToolkit.Urdf;
using UnityEngine;
using UrdfToolkit.Vendor;

namespace UrdfToolkit.Urdf
{
public class UrdfRobot : MonoBehaviour
{
    public List<UrdfJoint> joints = new List<UrdfJoint>();
    public List<UrdfLink> links = new List<UrdfLink>();

    // Safe property with null check
    public UrdfLink RootLink => links.Count > 0 ? links[0] : null;

    // Find methods
    public UrdfJoint GetJoint(string name)
    {
        return joints.Find(j => j.name == name);
    }

    public UrdfLink GetLink(string name)
    {
        return links.Find(l => l.name == name);
    }

    // Get joint chain from root to specified link - essential for IK
    public List<UrdfJoint> GetJointChain(string endLinkName)
    {
        var endLink = GetLink(endLinkName);
        if (endLink == null) return new List<UrdfJoint>();

        return GetJointChain(endLink);
    }

    public List<UrdfJoint> GetJointChain(UrdfLink endLink)
    {
        var chain = new List<UrdfJoint>();
        var currentLink = endLink;

        // Walk up the chain to root
        while (currentLink != null && currentLink != RootLink)
        {
            // Find the joint that has this link as child
            var parentJoint = joints.Find(j => j.childLink == currentLink);
            if (parentJoint != null)
            {
                chain.Insert(0, parentJoint); // Insert at beginning to maintain order
                currentLink = parentJoint.parentLink;
            }
            else
            {
                break; // No parent joint found
            }
        }

        return chain;
    }

    // Get all joints that affect the position of a specific link
    public List<UrdfJoint> GetControllableJointChain(string endLinkName)
    {
        return GetJointChain(endLinkName).Where(j => !j.IsFixed).ToList();
    }

    // Forward kinematics - compute end effector position
    public Vector3 ComputeEndEffectorPosition(string endLinkName)
    {
        var endLink = GetLink(endLinkName);
        return endLink != null ? endLink.transform.position : Vector3.zero;
    }

    public Quaternion ComputeEndEffectorRotation(string endLinkName)
    {
        var endLink = GetLink(endLinkName);
        return endLink != null ? endLink.transform.rotation : Quaternion.identity;
    }

    public Pose ComputeEndEffectorPose(string endLinkName)
    {
        var endLink = GetLink(endLinkName);
        if (endLink == null) return new Pose(Vector3.zero, Quaternion.identity);

        return new Pose(endLink.transform.position, endLink.transform.rotation);
    }

    // Get joint positions for IK
    public float[] GetJointPositions()
    {
        return joints.Select(j => j.GetPosition()).ToArray();
    }

    public float[] GetJointPositions(List<UrdfJoint> jointChain)
    {
        return jointChain.Select(j => j.GetPosition()).ToArray();
    }

    // Set joint positions for IK
    public void SetJointPositions(float[] positions)
    {
        for (int i = 0; i < Mathf.Min(positions.Length, joints.Count); i++)
        {
            joints[i].SetPositionClamped(positions[i]);
        }
    }

    public void SetJointPositions(List<UrdfJoint> jointChain, float[] positions)
    {
        for (int i = 0; i < Mathf.Min(positions.Length, jointChain.Count); i++)
        {
            jointChain[i].SetPositionClamped(positions[i]);
        }
    }

    // Get joint limits - useful for IK constraints
    public (float[] lower, float[] upper) GetJointLimits()
    {
        var lower = joints.Select(j => (float)j.LowerLimit).ToArray();
        var upper = joints.Select(j => (float)j.UpperLimit).ToArray();
        return (lower, upper);
    }

    public (float[] lower, float[] upper) GetJointLimits(List<UrdfJoint> jointChain)
    {
        var lower = jointChain.Select(j => (float)j.LowerLimit).ToArray();
        var upper = jointChain.Select(j => (float)j.UpperLimit).ToArray();
        return (lower, upper);
    }

    // ---- Physics (ArticulationBody) ----

    [Header("Physics")]
    [SerializeField] private ArticulationDriveSettings driveSettings = new ArticulationDriveSettings
    {
        stiffness = 100000f,
        damping = 10000f,
        forceLimit = float.MaxValue,
    };

    /// <summary>
    /// True while an ArticulationBody chain exists (built by <see cref="EnablePhysics"/>). Derived
    /// from the components rather than stored, so it stays correct across Play-mode entry.
    /// </summary>
    public bool PhysicsEnabled => GetComponentInChildren<ArticulationBody>(true) != null;

    /// <summary>
    /// Switches the robot from kinematic transform control to dynamic ArticulationBody simulation.
    /// Adds an ArticulationBody to each link (root → leaves), configures each joint from the imported
    /// URDF data, and flips every joint into Dynamic mode. Control (manual/IK) keeps going through
    /// SetPosition, which now sets drive targets instead of transforms.
    /// </summary>
    [Button("Enable Physics")]
    public void EnablePhysics()
    {
        // GetComponentsInChildren returns parents before children, so ArticulationBodies are added
        // top-down — required, since each body links to the nearest ancestor ArticulationBody.
        foreach (var link in GetComponentsInChildren<UrdfLink>())
        {
            var body = link.GetComponent<ArticulationBody>();
            if (body == null) body = link.gameObject.AddComponent<ArticulationBody>();

            ApplyInertial(link, body);

            if (link.joint != null)
            {
                link.joint.ConfigureArticulation(body, driveSettings);
                link.joint.driveMode = UrdfJointDriveMode.Dynamic;
            }
            else
            {
                // Root / base link: an immovable fixed base for the articulation.
                body.immovable = true;
            }
        }
    }

    /// <summary>
    /// Returns the robot to kinematic transform control: flips joints back to Kinematic mode and
    /// removes the ArticulationBodies. Transforms stay where physics left them; the next SetPosition
    /// recomposes correctly from the stored rest pose.
    /// </summary>
    [Button("Disable Physics")]
    public void DisablePhysics()
    {
        foreach (var joint in joints)
        {
            joint.driveMode = UrdfJointDriveMode.Kinematic;
        }

        foreach (var body in GetComponentsInChildren<ArticulationBody>())
        {
            if (Application.isPlaying) Destroy(body);
            else DestroyImmediate(body);
        }
    }

    private static void ApplyInertial(UrdfLink link, ArticulationBody body)
    {
        if (link.mass > 0f) body.mass = link.mass;
        if (link.hasCenterOfMass)
        {
            body.automaticCenterOfMass = false;
            body.centerOfMass = link.centerOfMass;
        }
        // NOTE: the inertia tensor is left to Unity's auto-compute (from colliders). Importing the
        // full URDF inertia tensor would require diagonalising it into principal axes — a follow-up.
    }

    // Collision and visual controls
    [Button("Disable Colliders")]
    public void DisableColliders()
    {
        var colliders = GetComponentsInChildren<Collider>();
        foreach (var collider in colliders)
        {
            collider.enabled = false;
        }
    }

    [Button("Enable Colliders")]
    public void EnableColliders()
    {
        var colliders = GetComponentsInChildren<Collider>();
        foreach (var collider in colliders)
        {
            collider.enabled = true;
        }
    }

    public void DisableVisuals()
    {
        var visuals = GetComponentsInChildren<MeshRenderer>();
        foreach (var visual in visuals)
        {
            visual.enabled = false;
        }
    }

    public void EnableVisuals()
    {
        var visuals = GetComponentsInChildren<MeshRenderer>();
        foreach (var visual in visuals)
        {
            visual.enabled = true;
        }
    }

    [Button("Setup IK")]
    public void SetupIK()
    {
        var solver = GetComponent<UrdfIKSolver>();
        if (!solver)
        {
            solver = gameObject.AddComponent<UrdfIKSolver>();
        }
        UrdfIKController controller = GetComponent<UrdfIKController>();
        if (!controller)
        {
            controller = gameObject.AddComponent<UrdfIKController>();
        }

        // create target at end effector
        // use the end of the chain by default
        var endEffector = joints.First(e => e is UrdfJointFixed);
        if (!endEffector) endEffector = joints.Last();
        if (endEffector != null)
        {
            var target = new GameObject("IK Target");
            target.transform.SetPositionAndRotation(endEffector.transform.position, endEffector.transform.rotation);
            controller.endEffectorLinkName = endEffector.name;
            controller.target = target.transform;
        }
    }

    // Fixed the swapped method names and added validation
    [ContextMenu("Populate Links")]
    public void PopulateLinks()
    {
        links.Clear();
        var foundLinks = GetComponentsInChildren<UrdfLink>();
        links.AddRange(foundLinks);
        Debug.Log($"Found {links.Count} links");
    }

    [ContextMenu("Populate Joints")]
    public void PopulateJoints()
    {
        joints.Clear();
        var foundJoints = GetComponentsInChildren<UrdfJoint>();
        joints.AddRange(foundJoints);
        Debug.Log($"Found {joints.Count} joints");
    }

    [ContextMenu("Populate All")]
    public void PopulateAll()
    {
        PopulateLinks();
        PopulateJoints();
        ValidateStructure();
    }

    // Validation method for debugging
    public bool ValidateStructure()
    {
        bool isValid = true;

        // Check if we have a root link
        if (RootLink == null)
        {
            Debug.LogError("No root link found!");
            isValid = false;
        }

        // Check joint-link relationships
        foreach (var joint in joints)
        {
            if (joint.parentLink == null)
            {
                Debug.LogWarning($"Joint {joint.name} has no parent link");
            }
            if (joint.childLink == null)
            {
                Debug.LogWarning($"Joint {joint.name} has no child link");
            }
        }

        Debug.Log($"Robot structure validation: {(isValid ? "PASSED" : "FAILED")}");
        return isValid;
    }

    // Pose structure for convenience
    [System.Serializable]
    public struct Pose
    {
        public Vector3 position;
        public Quaternion rotation;

        public Pose(Vector3 pos, Quaternion rot)
        {
            position = pos;
            rotation = rot;
        }
    }
}
}
