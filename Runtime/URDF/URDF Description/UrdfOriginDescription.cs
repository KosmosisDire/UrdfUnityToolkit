using UnityEngine;
using System.Xml;
using UrdfToolkit.Extensions;

namespace UrdfToolkit.Urdf
{

public struct UrdfOriginDef
{
    /// <summary>
    /// Right, Up, Forward <br/>
    /// Used in the Unity coordinate system.
    /// </summary>
    public Vector3 xyzRUF;

    /// <summary>
    /// Right, Up, Forward <br/>
    /// Used in the Unity coordinate system.
    /// </summary>
    public Quaternion rotationRUF;

    public Vector3 xyz;
    public Vector3 rpy;


    public UrdfOriginDef(XmlNode source)
    {
        var xyzValues = source.Attributes?["xyz"]?.Value.Split(' ');
        var rpyValues = source.Attributes?["rpy"]?.Value.Split(' ');

        if (xyzValues?.Length != 3)
        {
            xyz = Vector3.zero;
            xyzRUF = Vector3.zero;
        }
        else
        {
            xyz = new Vector3(
                    xyzValues[0].ParseFloatOrDefault(0),
                    xyzValues[1].ParseFloatOrDefault(0),
                    xyzValues[2].ParseFloatOrDefault(0));

            xyzRUF = xyz.Ros2Unity();
        }

        if (rpyValues?.Length != 3)
        {
            rpy = Vector3.zero;
            rotationRUF = Quaternion.identity;
        }
        else
        {
            rpy = new Vector3(
                    rpyValues[0].ParseFloatOrDefault(0),
                    rpyValues[1].ParseFloatOrDefault(0),
                    rpyValues[2].ParseFloatOrDefault(0));

            // URDF rpy (radians) is a fixed-axis Rz(yaw)·Ry(pitch)·Rx(roll) rotation in ROS space.
            // Build that rotation as a quaternion, then convert it to Unity's frame with the same
            // basis change used for positions. NOTE: a per-component Euler reorder/negate shortcut is
            // only correct for single-axis rotations; it mangles combined rpy (e.g. a pure ROS yaw
            // ends up as a rotation about the wrong axis), which left arm links disconnected.
            rotationRUF = RpyToQuaternion(rpy).Ros2Unity();
        }
    }

    /// <summary>
    /// Converts URDF roll/pitch/yaw (radians, applied as Rz(yaw)·Ry(pitch)·Rx(roll)) into a
    /// quaternion in ROS coordinates, using the standard URDF/tf formula. Convert the result with
    /// <see cref="RosExtensions.Ros2Unity(Quaternion)"/> to get a Unity-frame rotation.
    /// </summary>
    private static Quaternion RpyToQuaternion(Vector3 rpy)
    {
        float cr = Mathf.Cos(rpy.x * 0.5f), sr = Mathf.Sin(rpy.x * 0.5f);
        float cp = Mathf.Cos(rpy.y * 0.5f), sp = Mathf.Sin(rpy.y * 0.5f);
        float cy = Mathf.Cos(rpy.z * 0.5f), sy = Mathf.Sin(rpy.z * 0.5f);

        return new Quaternion(
            sr * cp * cy - cr * sp * sy,   // x
            cr * sp * cy + sr * cp * sy,   // y
            cr * cp * sy - sr * sp * cy,   // z
            cr * cp * cy + sr * sp * sy);  // w
    }

    public readonly string Stringify(int indentation)
    {
        return $"xyz: {xyz}  rpy: {rpy}".Indent(indentation);
    }
}

}