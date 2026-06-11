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
    public class UrdfJointRevolute : UrdfJoint
    {
        // public override UrdfJointType JointType => UrdfJointType.Revolute;

        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointRevolute urdfJoint = linkObject.AddComponent<UrdfJointRevolute>();
            return urdfJoint;
        }

        #region Runtime

        /// <summary>
        /// Returns the current position of the joint in degrees
        /// </summary>
        /// <returns>floating point number for joint position in degrees</returns>
        protected override float GetKinematicPosition()
        {
            // Strip the baseline origin rotation to get the joint's motion, then measure its angle
            // about the joint axis using a reference vector guaranteed not parallel to that axis.
            Quaternion motion = Quaternion.Inverse(originRotation) * transform.localRotation;
            Vector3 reference = Vector3.Cross(LocalAxis, Vector3.up);
            if (reference.sqrMagnitude < 1e-6f)
                reference = Vector3.Cross(LocalAxis, Vector3.right);
            return Vector3.SignedAngle(reference, motion * reference, LocalAxis);
        }

        /// <summary>
        /// Sets the target position of the joint in degrees
        /// </summary>
        /// <param name="position">Target position in degrees</param>
        protected override void SetKinematicPosition(float position)
        {
            // Compose the joint rotation on top of the rest pose instead of replacing it.
            transform.localRotation = originRotation * Quaternion.AngleAxis(position, LocalAxis);
        }

        /// <summary>
        /// Configures the ArticulationBody as a revolute joint about <see cref="UrdfJoint.LocalAxis"/>,
        /// with the URDF limits as drive limits (degrees) or free motion for a continuous joint.
        /// </summary>
        public override void ConfigureArticulation(ArticulationBody body, ArticulationDriveSettings gains)
        {
            articulationBody = body;
            body.jointType = ArticulationJointType.RevoluteJoint;

            // Define BOTH anchor frames explicitly. With matchAnchors (default) Unity derives the
            // parent anchor from whatever pose the link is in when the body is configured, making
            // jointPosition zero THERE — an enable-pose-relative frame nothing else in the system
            // uses. Pinning the parent anchor to the joint <origin> instead makes the articulation
            // constraint
            //   childLocal = parentAnchor * RotX(q) * childAnchor⁻¹
            //              = TRS(originPosition, originRotation * AngleAxis(q, LocalAxis))
            // — exactly SetKinematicPosition(q). So jointPosition, drive targets and drive limits are
            // ALWAYS in import-rest units (the same space as the kinematic backend, the analytical FK
            // and the URDF limits), no matter when or in what pose physics is enabled.
            body.matchAnchors = false;
            Quaternion axisAlignment = Quaternion.FromToRotation(Vector3.right, LocalAxis);
            body.anchorPosition = Vector3.zero;        // pivot = the link origin (the URDF joint frame)
            body.anchorRotation = axisAlignment;       // the articulation revolute spins about anchor X
            body.parentAnchorPosition = originPosition;
            body.parentAnchorRotation = originRotation * axisAlignment;

            var drive = body.xDrive;
            drive.stiffness = gains.stiffness;
            drive.damping = gains.damping;
            drive.forceLimit = gains.forceLimit;
            if (HasLimits)
            {
                body.twistLock = ArticulationDofLock.LimitedMotion;
                drive.lowerLimit = (float)LowerLimit;   // degrees, import-rest frame
                drive.upperLimit = (float)UpperLimit;
            }
            else
            {
                body.twistLock = ArticulationDofLock.FreeMotion;   // continuous
            }
            // Hold the pose the joint is in right now — not 0, which would snap a posed arm back to
            // rest the moment physics starts.
            drive.target = GetKinematicPosition();
            body.xDrive = drive;
        }

        // ArticulationBody reports angular jointPosition in radians; our convention is degrees.
        // (Explicit anchors put jointPosition in the import-rest frame — see ConfigureArticulation.)
        protected override float GetDynamicPosition()
        {
            var positions = articulationBody.jointPosition;
            return positions.dofCount > 0 ? positions[0] * Mathf.Rad2Deg : 0f;
        }

        public override Matrix4x4 LocalMotionMatrix(float jointValue)
        {
            // Mirrors SetKinematicPosition: rotate about the axis on top of the rest pose.
            return Matrix4x4.TRS(originPosition, originRotation * Quaternion.AngleAxis(jointValue, LocalAxis), Vector3.one);
        }

        /// <summary>
        /// Get the transform matrix for this revolute joint
        /// </summary>
        public override Matrix4x4 GetJointTransform()
        {
            float angle = GetPosition();
            return Matrix4x4.Rotate(Quaternion.AngleAxis(angle, LocalAxis));
        }

        #endregion

        protected override void ImportJointData(UrdfJointDef joint)
        {
            base.ImportJointData(joint);

            // Continuous joints spin freely: their <limit> only carries effort/velocity, so leave
            // Lower/UpperLimit at ±infinity (base.ImportJointData already skips them).
            if (joint.type != UrdfJointType.Continuous && joint.limit != null)
            {
                // URDF angles are radians; this joint API works in degrees (see SetPosition), so convert.
                LowerLimit = joint.limit.Value.lower * Mathf.Rad2Deg;
                UpperLimit = joint.limit.Value.upper * Mathf.Rad2Deg;
            }

            // Unity rotates left-handed while URDF/ROS is right-handed, so negate the converted
            // axis to keep a positive joint angle turning the correct way. (Prismatic joints
            // translate and don't need this flip.)
            axisofMotion = -joint.axis?.xyzRUF ?? -Vector3.right;
        }
    }
}