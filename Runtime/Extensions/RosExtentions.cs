using UnityEngine;

namespace UrdfToolkit.Extensions
{
    public static class RosExtensions
    {
        public static Vector3 Ros2Unity(this Vector3 vector3)
        {
            return new Vector3(-vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2Ros(this Vector3 vector3)
        {
            return new Vector3(vector3.z, -vector3.x, vector3.y);
        }

        public static Quaternion Ros2Unity(this Quaternion quaternion)
        {
            return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
        }

        public static Quaternion Unity2Ros(this Quaternion quaternion)
        {
            return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
        }

        public static Vector3 Ros2UnityScale(this Vector3 vector3)
        {
            return new Vector3(vector3.y, vector3.z, vector3.x);
        }

        public static Vector3 Unity2RosScale(this Vector3 vector3)
        {
            return new Vector3(vector3.z, vector3.x, vector3.y);
        }
    }
}