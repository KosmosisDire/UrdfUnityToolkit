/*
© Siemens AG, 2017-2019
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
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

using UrdfToolkit.Urdf.Importer;
using UrdfToolkit.Urdf;
using UnityEngine;
using UrdfToolkit.Vendor;

namespace UrdfToolkit.Urdf
{
    /// <summary>How a joint's pose is driven each frame.</summary>
    public enum UrdfJointDriveMode
    {
        /// <summary>The transform is set directly, composed onto the rest pose. No physics.</summary>
        Kinematic,

        /// <summary>An ArticulationBody simulates the joint; control sets a PD drive target.</summary>
        Dynamic,
    }

    /// <summary>PD-drive gains applied to a joint's ArticulationBody in Dynamic mode.</summary>
    [System.Serializable]
    public struct ArticulationDriveSettings
    {
        public float stiffness;
        public float damping;
        public float forceLimit;

        public static ArticulationDriveSettings Default => new ArticulationDriveSettings
        {
            stiffness = 100000f,
            damping = 10000f,
            forceLimit = float.MaxValue,
        };
    }

    [ExecuteAlways]
    public abstract class UrdfJoint : MonoBehaviour
    {
        public UrdfLink parentLink;
        public UrdfLink childLink;
        public Vector3 axisofMotion;

        // Active drive backend. Runtime state, not serialized — re-derived in OnEnable from whether
        // an ArticulationBody component is present, so it survives Play-mode entry / domain reloads.
        [System.NonSerialized] public UrdfJointDriveMode driveMode = UrdfJointDriveMode.Kinematic;
        protected ArticulationBody articulationBody;
        public ArticulationBody ArticulationBody => articulationBody;

        // Rest pose relative to the parent (the joint <origin>, applied by the builder). Joint
        // motion is composed on top of this baseline instead of overwriting the local transform.
        // Serialized: it is captured once at import, and the kinematic backend, the analytical FK
        // and the articulation anchors all derive from it — losing it on Play-mode entry collapses
        // every joint onto its parent.
        [SerializeField, HideInInspector] protected Vector3 originPosition = Vector3.zero;
        [SerializeField, HideInInspector] protected Quaternion originRotation = Quaternion.identity;

        protected virtual void OnEnable()
        {
            // Re-derive the drive backend after deserialization (Play-mode entry, domain reload,
            // scene load): the ArticulationBody component survives those, the fields above don't.
            articulationBody = GetComponent<ArticulationBody>();
            driveMode = articulationBody != null ? UrdfJointDriveMode.Dynamic : UrdfJointDriveMode.Kinematic;
        }

        // Joint limits for IK
        public double LowerLimit = double.NegativeInfinity;
        public double UpperLimit = double.PositiveInfinity;
        public bool HasLimits => !double.IsInfinity(LowerLimit) && !double.IsInfinity(UpperLimit);
        public bool IsFixed => this is UrdfJointFixed;

        // Properties for IK
        public Vector3 WorldAxis => transform.TransformDirection(axisofMotion);
        public Vector3 LocalAxis => axisofMotion;

        public static UrdfJoint Create(GameObject linkObject, UrdfJointDef joint)
        {
            UrdfJoint urdfJoint = AddCorrectJointType(linkObject, joint.type);
            urdfJoint.ImportJointData(joint);
            return urdfJoint;
        }

        private static UrdfJoint AddCorrectJointType(GameObject linkObject, UrdfJointType jointType)
        {
            UrdfJoint urdfJoint = null;

            switch (jointType)
            {
                case UrdfJointType.Revolute:
                case UrdfJointType.Continuous:
                    // Continuous is a revolute joint without rotation limits.
                    urdfJoint = UrdfJointRevolute.Create(linkObject);
                    break;
                case UrdfJointType.Prismatic:
                    urdfJoint = UrdfJointPrismatic.Create(linkObject);
                    break;
                case UrdfJointType.Fixed:
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                case UrdfJointType.Floating:
                case UrdfJointType.Planar:
                    // Multi-DOF joints (6-DOF free / 3-DOF planar) don't fit the single-axis joint
                    // model, so we import them as Fixed: the connected links stay rigid. They are
                    // almost always world/base attachments rather than actuated joints.
                    Debug.LogWarning($"[urdf] joint type '{jointType}' is multi-DOF and not supported; importing it as Fixed (the connected links will be rigid).");
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
                default:
                    Debug.LogWarning($"[urdf] unknown joint type '{jointType}'; importing it as Fixed.");
                    urdfJoint = UrdfJointFixed.Create(linkObject);
                    break;
            }

            return urdfJoint;
        }

        /// <summary>
        /// Changes the type of the joint
        /// </summary>
        /// <param name="linkObject">Joint whose type is to be changed</param>
        /// <param name="newJointType">Type of the new joint</param>
        public static void ChangeJointType(GameObject linkObject, UrdfJointType newJointType)
        {
            linkObject.DestroyImmediateIfExists<UrdfJoint>();
            linkObject.DestroyImmediateIfExists<PrismaticJointLimitsManager>();
            linkObject.DestroyImmediateIfExists<ArticulationBody>();
            AddCorrectJointType(linkObject, newJointType);
        }

        #region Runtime

        /// <summary>
        /// Current joint value (degrees for revolute, metres for prismatic), read from whichever
        /// backend is active: the transform in Kinematic mode, the ArticulationBody in Dynamic mode.
        /// </summary>
        public float GetPosition()
        {
            // jointPosition is only computed while the body simulates (Play mode); outside that,
            // measuring the live transform against the rest pose is always valid.
            if (driveMode == UrdfJointDriveMode.Dynamic && articulationBody != null && Application.isPlaying)
                return GetDynamicPosition();
            return GetKinematicPosition();
        }

        /// <summary>
        /// Canonical entry point for driving a joint to a target value (degrees for revolute, metres
        /// for prismatic). Routes to the active backend: Kinematic composes the motion onto the rest
        /// pose (<see cref="originPosition"/>/<see cref="originRotation"/>); Dynamic sets the
        /// ArticulationBody drive target (a PD setpoint). ALL control — manual, IK, animation — must go
        /// through this (or <see cref="SetPositionClamped"/> /
        /// <see cref="UrdfRobot.SetJointPositions(float[])"/>). Never set the joint's
        /// <c>transform.local*</c> directly; that bypasses both the origin and the physics backend.
        /// </summary>
        public void SetPosition(float position)
        {
            if (driveMode == UrdfJointDriveMode.Dynamic && articulationBody != null)
                SetDynamicTarget(position);
            else
                SetKinematicPosition(position);
        }

        public virtual void SetPositionClamped(float position)
        {
            if (HasLimits)
            {
                position = Mathf.Clamp(position, (float)LowerLimit, (float)UpperLimit);
            }
            SetPosition(position);
        }

        // Get the joint's contribution to transform hierarchy
        public virtual Matrix4x4 GetJointTransform()
        {
            return transform.localToWorldMatrix;
        }

        /// <summary>
        /// The joint's local transform relative to its parent link for a given joint value — the
        /// analytical equivalent of what the Kinematic backend would write to the transform. The IK
        /// solver uses this to evaluate forward kinematics in math, so it works even while an
        /// ArticulationBody owns the live transforms. Base/fixed = the rest pose; revolute and
        /// prismatic override it with their motion.
        /// </summary>
        public virtual Matrix4x4 LocalMotionMatrix(float jointValue)
        {
            return Matrix4x4.TRS(originPosition, originRotation, Vector3.one);
        }

        #endregion

        #region Backends

        // --- Kinematic (transform) backend: composes the joint motion onto the rest pose. ---
        protected abstract float GetKinematicPosition();
        protected abstract void SetKinematicPosition(float position);

        // --- Dynamic (ArticulationBody) backend. ---

        /// <summary>
        /// Configures this joint's ArticulationBody (joint type, axis/anchor, limits, drive) from the
        /// imported URDF data and stores it for Dynamic-mode control. The robot calls this when
        /// enabling physics. Base implementation handles the no-DOF (fixed) case.
        /// </summary>
        public virtual void ConfigureArticulation(ArticulationBody body, ArticulationDriveSettings gains)
        {
            articulationBody = body;
            body.jointType = ArticulationJointType.FixedJoint;
        }

        /// <summary>
        /// Reads the joint value from the ArticulationBody. Because ConfigureArticulation pins BOTH
        /// anchors explicitly (parent anchor = the joint origin), jointPosition is measured from the
        /// import rest pose — the same frame as the kinematic backend. Override per type for units.
        /// </summary>
        protected virtual float GetDynamicPosition()
        {
            var positions = articulationBody.jointPosition;
            return positions.dofCount > 0 ? positions[0] : 0f;
        }

        /// <summary>Sets the ArticulationBody drive target (degrees for revolute, metres for prismatic).</summary>
        protected virtual void SetDynamicTarget(float position)
        {
            var drive = articulationBody.xDrive;
            drive.target = position;
            articulationBody.xDrive = drive;
        }

        #endregion

        #region Import Helpers

        protected virtual void ImportJointData(UrdfJointDef joint)
        {
            // Capture the rest pose the builder placed on our transform (the joint <origin>). Joint
            // motion is applied relative to this, so SetPosition(0) leaves the link exactly where the
            // URDF positioned it instead of collapsing it onto the parent.
            originPosition = transform.localPosition;
            originRotation = transform.localRotation;

            // Continuous joints rotate without limit; any <limit> they carry is effort/velocity only.
            if (joint.type != UrdfJointType.Continuous && joint.limit != null)
            {
                LowerLimit = joint.limit.Value.lower;
                UpperLimit = joint.limit.Value.upper;
            }
        }

        protected virtual void AdjustMovement(UrdfJointDef joint) { }

        #endregion
    }
}