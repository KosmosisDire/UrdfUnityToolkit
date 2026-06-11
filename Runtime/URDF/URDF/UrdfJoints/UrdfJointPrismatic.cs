/*
© Siemens AG, 2018-2019
Author: Suzannah Smith (suzannah.smith@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UrdfToolkit.Urdf;
using UnityEngine;

namespace UrdfToolkit.Urdf
{
    public class UrdfJointPrismatic : UrdfJoint
    {
        private ArticulationDrive drive;
#if UNITY_2020_1
        private float maxLinearVelocity;
#endif

        // public override UrdfJointType JointType => UrdfJointType.Prismatic;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointPrismatic urdfJoint = linkObject.AddComponent<UrdfJointPrismatic>();


            return urdfJoint;
        }

        #region Runtime

        /// <summary>
        /// Returns the current position of the joint in meters
        /// </summary>
        /// <returns>floating point number for joint position in meters</returns>
        protected override float GetKinematicPosition()
        {
            // Displacement from the rest pose, brought back into the joint frame, projected onto the axis.
            Vector3 offset = Quaternion.Inverse(originRotation) * (transform.localPosition - originPosition);
            return Vector3.Dot(offset, LocalAxis);
        }

        /// <summary>
        /// Sets the target position of the joint in meters
        /// </summary>
        /// <param name="position">Target position in meters</param>
        protected override void SetKinematicPosition(float position)
        {
            // Slide along the axis (in the joint frame) starting from the rest pose, instead of
            // overwriting the origin offset the builder set.
            transform.localPosition = originPosition + originRotation * (LocalAxis * position);
        }

        /// <summary>
        /// Configures the ArticulationBody as a prismatic joint sliding along
        /// <see cref="UrdfJoint.LocalAxis"/>, with the URDF limits as drive limits (metres).
        /// </summary>
        public override void ConfigureArticulation(ArticulationBody body, ArticulationDriveSettings gains)
        {
            articulationBody = body;
            body.jointType = ArticulationJointType.PrismaticJoint;

            // Define BOTH anchor frames explicitly so jointPosition is measured from the import rest
            // pose rather than the enable-time pose (see UrdfJointRevolute.ConfigureArticulation for
            // the derivation):
            //   childLocal = parentAnchor * Trans(q·X) * childAnchor⁻¹
            //              = TRS(originPosition + originRotation * (LocalAxis * q), originRotation)
            // — exactly SetKinematicPosition(q), so targets/readback/limits stay in import-rest metres.
            body.matchAnchors = false;
            Quaternion axisAlignment = Quaternion.FromToRotation(Vector3.right, LocalAxis);
            body.anchorPosition = Vector3.zero;
            body.anchorRotation = axisAlignment;   // the articulation prismatic slides along anchor X
            body.parentAnchorPosition = originPosition;
            body.parentAnchorRotation = originRotation * axisAlignment;
            body.linearLockY = ArticulationDofLock.LockedMotion;
            body.linearLockZ = ArticulationDofLock.LockedMotion;

            var drive = body.xDrive;
            drive.stiffness = gains.stiffness;
            drive.damping = gains.damping;
            drive.forceLimit = gains.forceLimit;
            if (HasLimits)
            {
                body.linearLockX = ArticulationDofLock.LimitedMotion;
                drive.lowerLimit = (float)LowerLimit;   // metres, import-rest frame
                drive.upperLimit = (float)UpperLimit;
            }
            else
            {
                body.linearLockX = ArticulationDofLock.FreeMotion;
            }
            // Hold the pose the joint is in right now — not 0, which would snap a posed arm back to
            // rest the moment physics starts.
            drive.target = GetKinematicPosition();
            body.xDrive = drive;
        }

        // GetDynamicPosition: ArticulationBody jointPosition is already in metres for prismatic — the
        // base implementation is correct, so no override is needed.

        public override Matrix4x4 LocalMotionMatrix(float jointValue)
        {
            // Mirrors SetKinematicPosition: slide along the axis from the rest pose.
            return Matrix4x4.TRS(originPosition + originRotation * (LocalAxis * jointValue), originRotation, Vector3.one);
        }

        /// <summary>
        /// Get the transform matrix for this prismatic joint
        /// </summary>
        public override Matrix4x4 GetJointTransform()
        {
            float position = GetPosition();
            Vector3 translation = LocalAxis * position;
            return Matrix4x4.Translate(translation);
        }

        #endregion

        #region Import

        protected override void ImportJointData(UrdfJointDef joint)
        {
            base.ImportJointData(joint);

            // Prismatic limits are linear (metres), so they're used as-is — no rad→deg conversion.
            LowerLimit = joint.limit?.lower ?? float.NegativeInfinity;
            UpperLimit = joint.limit?.upper ?? float.PositiveInfinity;

            // Translation needs no handedness flip (unlike revolute rotation, which negates the
            // axis), so the converted axis is used directly: a positive joint value slides along +axis.
            axisofMotion = joint.axis?.xyzRUF ?? Vector3.forward;
        }

        #endregion

    }
}