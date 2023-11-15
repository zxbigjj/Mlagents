using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.MLAgents;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Used to store relevant information for acting and learning for each body part in agent.
    /// </summary>
    [System.Serializable]
    public class BodyPart
    {
        [Header("Body Part Info")][Space(10)] public ConfigurableJoint joint;
        public Rigidbody rb;
        [HideInInspector] public Vector3 startingPos;//���λ��
        [HideInInspector] public Quaternion startingRot;//�����ת

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;

        //public TargetContact targetContact;

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveController thisJdController;//��ǰ��JointDriveController

        [Header("Current Joint Settings")]
        [Space(10)]
        public Vector3 currentEularJointRotation;//��ǰŷ����ת

        [HideInInspector] public float currentStrength;
        public float currentXNormalizedRot;
        public float currentYNormalizedRot;
        public float currentZNormalizedRot;

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;//������Ϊ��������Լ��(��ǰ��Ϊ�ǹؽ�)��ʩ�ӵ���

        public float currentJointForceSqrMag;//����������ĳ���
        public Vector3 currentJointTorque;//������Ϊ��������Լ����ʩ�ӵ�Ť��(��ת����)
        public float currentJointTorqueSqrMag;//�����ת�������ĳ���
        public AnimationCurve jointForceCurve = new AnimationCurve();//���챣��ĳʱ������ʱ����(��ȷ��)
        public AnimationCurve jointTorqueCurve = new AnimationCurve();//���챣��ĳʱ����ת����ʱ����(��ȷ��)

        /// <summary>
        /// Reset body part to initial configuration.
        /// </summary>
        public void Reset(BodyPart bp)
        {
            bp.rb.transform.position = bp.startingPos;//
            bp.rb.transform.rotation = bp.startingRot;
            bp.rb.velocity = Vector3.zero;
            bp.rb.angularVelocity = Vector3.zero;
            if (bp.groundContact)
            {
                bp.groundContact.touchingGround = false;
            }

            //if (bp.targetContact)
            //{
            //    bp.targetContact.touchingTarget = false;
            //}
        }

        /// <summary>
        /// Apply torque according to defined goal `x, y, z` angle and force `strength`.
        /// </summary>
        public void SetJointTargetRotation(float x, float y, float z)
        {
            x = (x + 1f) * 0.5f;//��ʼλ��Ϊ�е�0.5
            y = (y + 1f) * 0.5f;
            z = (z + 1f) * 0.5f;

            var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);//��ȡ��������֮�� �ٷ�֮x��ֵ
            var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
            var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

            currentXNormalizedRot =
                Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, xRot);//����0-1֮��ֵ
            currentYNormalizedRot = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, yRot);
            currentZNormalizedRot = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, zRot);

            joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);//��ŷ������ת
            currentEularJointRotation = new Vector3(xRot, yRot, zRot);//���õ�ǰ��ת
        }

        public void SetJointStrength(float strength)//����Joint�ؽ�SlerpDrive����
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;
            var jd = new JointDrive
            {
                positionSpring = thisJdController.maxJointSpring,
                positionDamper = thisJdController.jointDampen,
                maximumForce = rawVal
            };
            joint.slerpDrive = jd;
            currentStrength = jd.maximumForce;
        }
    }

    public class JointDriveController : MonoBehaviour
    {
        [Header("Joint Drive Settings")]
        [Space(10)]
        public float maxJointSpring;

        public float jointDampen;
        public float maxJointForceLimit;

        [HideInInspector] public Dictionary<Transform, BodyPart> bodyPartsDict = new Dictionary<Transform, BodyPart>();

        [HideInInspector] public List<BodyPart> bodyPartsList = new List<BodyPart>();
        const float k_MaxAngularVelocity = 50.0f;

        /// <summary>
        /// Create BodyPart object and add it to dictionary.
        /// </summary>
        public void SetupBodyPart(Transform t)
        {
            var bp = new BodyPart
            {
                rb = t.GetComponent<Rigidbody>(),
                joint = t.GetComponent<ConfigurableJoint>(),
                startingPos = t.position,
                startingRot = t.rotation
            };
            bp.rb.maxAngularVelocity = k_MaxAngularVelocity;

            // Add & setup the ground contact script
            bp.groundContact = t.GetComponent<GroundContact>();
            if (!bp.groundContact)
            {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();
                bp.groundContact.agent = gameObject.GetComponent<Agent>();
            }
            else
            {
                bp.groundContact.agent = gameObject.GetComponent<Agent>();
            }

            if (bp.joint)
            {
                var jd = new JointDrive
                {
                    positionSpring = maxJointSpring,
                    positionDamper = jointDampen,
                    maximumForce = maxJointForceLimit
                };
                bp.joint.slerpDrive = jd;
            }

            bp.thisJdController = this;
            bodyPartsDict.Add(t, bp);
            bodyPartsList.Add(bp);
        }

        public void GetCurrentJointForces()
        {
            foreach (var bodyPart in bodyPartsDict.Values)
            {
                if (bodyPart.joint)
                {
                    bodyPart.currentJointForce = bodyPart.joint.currentForce;
                    bodyPart.currentJointForceSqrMag = bodyPart.joint.currentForce.magnitude;
                    bodyPart.currentJointTorque = bodyPart.joint.currentTorque;
                    bodyPart.currentJointTorqueSqrMag = bodyPart.joint.currentTorque.magnitude;
                    if (Application.isEditor)
                    {
                        if (bodyPart.jointForceCurve.length > 1000)
                        {
                            bodyPart.jointForceCurve = new AnimationCurve();
                        }

                        if (bodyPart.jointTorqueCurve.length > 1000)
                        {
                            bodyPart.jointTorqueCurve = new AnimationCurve();
                        }

                        bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag);
                        bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag);
                    }
                }
            }
        }
    }
}
