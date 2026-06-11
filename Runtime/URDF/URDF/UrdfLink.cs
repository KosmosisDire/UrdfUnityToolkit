using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

namespace UrdfToolkit.Urdf
{

public class UrdfLink : MonoBehaviour
{
    public bool isBaseLink;

    public List<UrdfLink> childLinks = new List<UrdfLink>();
    public UrdfJoint joint;
    public UrdfRobot robot;

    // Inertial data from the URDF <inertial> block, captured at build time. Used to configure the
    // ArticulationBody when physics is enabled (ignored in kinematic mode).
    public float mass = 0f;
    public bool hasCenterOfMass = false;
    public Vector3 centerOfMass = Vector3.zero;

    void Awake()
    {
        robot = GetComponentInParent<UrdfRobot>();
    }

    void OnDestroy()
    {
    }

}


}

