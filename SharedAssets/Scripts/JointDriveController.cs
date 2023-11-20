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
        [Header("Body Part Info")][Space(10)] public ConfigurableJoint joint;//�����ùؽ�
        public Rigidbody rb;//����ģ��
        [HideInInspector] public Vector3 startingPos;//���λ��
        [HideInInspector] public Quaternion startingRot;//�����ת

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;//�Ƿ�Ӵ������ź���

        //public TargetContact targetContact; ֮ǰ�������ж��Ƿ�Ӵ���ʳ��

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveController thisJdController;//��ǰ��JointDriveController

        [Header("Current Joint Settings")]
        [Space(10)]
        public Vector3 currentEularJointRotation;//��ǰ�����ùؽ�ŷ����ת �����Ǹ����Դ����ź�

        [HideInInspector] public float currentStrength;//m_SlerpDrive.maximumForce��ֵ�����Ǹ����Դ����ź�
        public float currentXNormalizedRot;//ת���� 0 -1float�źŵ� ��ǰ��ת�Ƕȿ����Ǹ����Դ����ź�
        public float currentYNormalizedRot;//ת���� 0 -1float�źŵ� ��ǰ��ת�Ƕȿ����Ǹ����Դ����ź�
        public float currentZNormalizedRot;//ת���� 0 -1float�źŵ� ��ǰ��ת�Ƕȿ����Ǹ����Դ����ź�

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;//������Ϊ��������Լ��(��ǰ��Ϊ�ǹؽ�)��ʩ�ӵ��� �����Ǳ������ؽ�Ӱ���
                                         //����������� ������

        public float currentJointForceSqrMag;//����������ĳ��� ����С
        public Vector3 currentJointTorque;//������Ϊ��������Լ����ʩ�ӵ�Ť��(��ת����) ��ת����С
        public float currentJointTorqueSqrMag;//�����ת�������ĳ��� ��ת������
        //public AnimationCurve jointForceCurve = new AnimationCurve();//���챣��ĳʱ������ʱ����(��ȷ��)
        //public AnimationCurve jointTorqueCurve = new AnimationCurve();//���챣��ĳʱ����ת����ʱ����(��ȷ��)

        /// <summary>
        /// Reset body part to initial configuration.
        /// </summary>
        public void Reset(BodyPart bp)//���ùؽ�Ϊ��ʼ״̬ ������������һ��ѵ��
        {
            bp.rb.transform.position = bp.startingPos;//��ǰ��λ�ø�ԭ�����λ��
            bp.rb.transform.rotation = bp.startingRot;//��ǰ����ת��ԭ�������ת
            bp.rb.velocity = Vector3.zero;//�ٶȹ���
            bp.rb.angularVelocity = Vector3.zero;//���ٶȹ���
            if (bp.groundContact)//����Ӵ������ź�Ϊ true �����ź�
            {
                bp.groundContact.touchingGround = false; //��Ϊfalse 
            }

            //if (bp.targetContact)
            //{
            //    bp.targetContact.touchingTarget = false;
            //}
        }

        /// <summary>
        /// Apply torque according to defined goal `x, y, z` angle and force `strength`.
        /// </summary>
        public void SetJointTargetRotation(float x, float y, float z)//���ܴ����źź� ������ת�ĽǶ�
        {
            x = (x + 1f) * 0.5f;//��ʼλ��Ϊ�е�0.5 �²� x y z �Ĵ�С��Χ��Ϊ [-1,1]
            y = (y + 1f) * 0.5f;//��ʼλ��Ϊ�е�0.5
            z = (z + 1f) * 0.5f;//��ʼλ��Ϊ�е�0.5
            //ȡ��ֵΪ (joint.highAngularXLimit.limit -  joint.lowAngularXLimit.limit) * x(0-1֮���ֵ) + joint.lowAngularXLimit.limit
            var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);//��ȡ��������֮��
                                                                                                  //�ٷ�֮x��ֵ
                                                                                                  //���� xΪ 0.5 ʱ
                                                                                                  //ȡ��ֵΪ 60-(-60) * 0.5 +(-60) =0
                                                                                                  //�� [0,1]��ɽǶ�ֵ[-60,60]

            var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
            var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

            currentXNormalizedRot =
                Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, xRot);//����0-1֮��ֵ
                                                                                                     //�� �Ƕ�ֵ[-60,60]���[0,1]
            currentYNormalizedRot = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, yRot);
            currentZNormalizedRot = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, zRot);

            joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);//��ŷ������ת ����m_TargetRotationĿ����ת����
            currentEularJointRotation = new Vector3(xRot, yRot, zRot);//���浱ǰ��ת
        }

        public void SetJointStrength(float strength)//����Joint�ؽ�SlerpDrive����m_SlerpDrive.maximumForce
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;//thisJdController.maxJointForceLimit
                                                                                      //�ٷ�����10000
            var jd = new JointDrive
            {
                positionSpring = thisJdController.maxJointSpring,//���� �ٷ�����3000
                positionDamper = thisJdController.jointDampen,//���� �ٷ�����30
                maximumForce = rawVal// ͨ����ʽȡ�õ�ǰ�����
            };
            joint.slerpDrive = jd; //��ֵ��slerpDrive
            currentStrength = jd.maximumForce;//��¼��ǰ�����
        }
    }

    public class JointDriveController : MonoBehaviour
    {
        [Header("Joint Drive Settings")]//ʵ������ʾ���� "JointDrive�������" 
        [Space(100)]//���ֺ��·������ľ���
        public float maxJointSpring;//����JointDrive�����ؽڵ���

        public float jointDampen;//����JointDrive��������
        public float maxJointForceLimit;//����JointDrive���������

        [HideInInspector] public Dictionary<Transform, BodyPart> bodyPartsDict = new Dictionary<Transform, BodyPart>();
                                                                //һ���ֵ� ��key��transform value��bodypart  

        [HideInInspector] public List<BodyPart> bodyPartsList = new List<BodyPart>();
                                                                //�б�(һά����) ��ÿ���ؽڵ�BodyPart���
        const float k_MaxAngularVelocity = 50.0f;//�����ٶ� maxAngularVelocity	������������ʣ��Ի���/��Ϊ��λ������Ĭ��ֵΪ 7����ΧΪ { 0, ����� }��
        //public AnimationCurve jointTorqueCurve;
        //public AnimationCurve jointForceCurve;
        /// <summary>
        /// Create BodyPart object and add it to dictionary.
        /// </summary>
        public void SetupBodyPart(Transform t) //��ʼ������BodyPart
        {
            var bp = new BodyPart 
            {
                rb = t.GetComponent<Rigidbody>(),//��ʼ����ֵ��transfrom���µ�Rigidbody���
                joint = t.GetComponent<ConfigurableJoint>(),//��ʼ����ֵ��transfrom���µ�joint���
                startingPos = t.position,//BodyPart��ʼλ��
                startingRot = t.rotation//BodyPart��ʼ��ת
            };
            bp.rb.maxAngularVelocity = k_MaxAngularVelocity; //����rigidbody����������

            // Add & setup the ground contact script
            bp.groundContact = t.GetComponent<GroundContact>();//��ʼ����ֵ��transfrom���µĵ���Ӵ����
            if (!bp.groundContact)//�����transform����û�е���Ӵ����
            {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();//��ô���һ������Ӵ�������ҳ�ʼ����ֵ
                bp.groundContact.agent = gameObject.GetComponent<Agent>();//��ʼ����ֵ
                                                                          //groundContact�������е�agent����
                                                                          //Ϊ���������jointDriveController��
                                                                          //��Ϸ������µ�Agent���
            }
            else//����groundContact����Ͳ��ø�����BodyPart����Ϸ����������
            {
                bp.groundContact.agent = gameObject.GetComponent<Agent>();//��ʼ����ֵ
                                                                          //groundContact�������е�agent����
                                                                          //Ϊ���������jointDriveController��
                                                                          //��Ϸ������µ�Agent���
            }

            if (bp.joint)//����йؽ�
            {
                var jd = new JointDrive//��ô�ؽڵĲ�����ʼ��
                {
                    positionSpring = maxJointSpring,//�����
                    positionDamper = jointDampen,//������
                    maximumForce = maxJointForceLimit//���������
                };
                bp.joint.slerpDrive = jd;//���˹ؽڵ�slerpDrive��ɳ�ʼ����jd
            }

            bp.thisJdController = this;//��BodyPart�ؽڴ��Ĳ���thisJdController ���� ����
            bodyPartsDict.Add(t, bp);//������ֵ�
            bodyPartsList.Add(bp);//������б�
        }

        //public void GetCurrentJointForces()
        //{
        //    foreach (var bodyPart in bodyPartsDict.Values)//���������ֵ��BodyPart
        //    {
        //        if (bodyPart.joint)//����йؽ�
        //        {
        //            bodyPart.currentJointForce = bodyPart.joint.currentForce;//����currentForce������currentJointForce
        //            bodyPart.currentJointForceSqrMag = bodyPart.joint.currentForce.magnitude;//����currentForce.magnitude������currentJointForceSqrMag
        //            bodyPart.currentJointTorque = bodyPart.joint.currentTorque;//����joint.currentTorque������currentJointTorque
        //            bodyPart.currentJointTorqueSqrMag = bodyPart.joint.currentTorque.magnitude;//����.currentTorque.magnitude������currentJointTorqueSqrMag
        //            if (Application.isEditor)//������ڱ༭��״̬������
        //            {
        //                if (bodyPart.jointForceCurve.length > 1000)//����1000 ��������ʱ���᳤�ȳ���1000���¼�һ�����±���
        //                {
        //                    bodyPart.jointForceCurve = new AnimationCurve();//AnimationCurve�������ʱ�������� Keyframes �ļ��ϡ�
        //
        //
        //                }
        //
        //                if (bodyPart.jointTorqueCurve.length > 1000)//������ת����ʱ���᳤�ȳ���1000���¼�һ�����±���
        //                {
        //                    bodyPart.jointTorqueCurve = new AnimationCurve();
        //                }
        //
        //                bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag);//���浱ǰʱ������
        //                bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag);//���浱ǰʱ������ת��
        //            }
        //        }
        //    }
        //}
    }
}
