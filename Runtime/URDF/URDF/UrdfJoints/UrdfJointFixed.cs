/*
© Siemens AG, 2018
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
    public class UrdfJointFixed : UrdfJoint
    {
        public static UrdfJoint Create(GameObject linkObject)
        {
            UrdfJointFixed urdfJoint = linkObject.AddComponent<UrdfJointFixed>();
            return urdfJoint;
        }

        #region Runtime

        /// <summary>
        /// Fixed joints have no position - always returns 0
        /// </summary>
        /// <returns>Always 0</returns>
        protected override float GetKinematicPosition()
        {
            return 0f;
        }

        /// <summary>
        /// Fixed joints cannot be positioned - does nothing
        /// </summary>
        /// <param name="position">Ignored</param>
        protected override void SetKinematicPosition(float position)
        {
            // Fixed joints cannot be moved
        }

        /// <summary>
        /// Fixed joints contribute identity transform
        /// </summary>
        public override Matrix4x4 GetJointTransform()
        {
            return Matrix4x4.identity;
        }

        #endregion

        protected override void ImportJointData(UrdfJointDef joint)
        {
            base.ImportJointData(joint);
            
            LowerLimit = 0;
            UpperLimit = 0;
            axisofMotion = Vector3.zero;
        }
    }
}