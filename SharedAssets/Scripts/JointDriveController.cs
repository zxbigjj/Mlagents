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
        [Header("Body Part Info")][Space(10)] public ConfigurableJoint joint;//可配置关节
        public Rigidbody rb;//物理模拟
        [HideInInspector] public Vector3 startingPos;//最初位置
        [HideInInspector] public Quaternion startingRot;//最初旋转

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;//是否接触地面信号类

        //public TargetContact targetContact; 之前是用来判断是否接触到食物

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveController thisJdController;//当前的JointDriveController

        [Header("Current Joint Settings")]
        [Space(10)]
        public Vector3 currentEularJointRotation;//当前可配置关节欧拉旋转 可能是给大脑传递信号

        [HideInInspector] public float currentStrength;//m_SlerpDrive.maximumForce的值可能是给大脑传递信号
        public float currentXNormalizedRot;//转换成 0 -1float信号的 当前旋转角度可能是给大脑传递信号
        public float currentYNormalizedRot;//转换成 0 -1float信号的 当前旋转角度可能是给大脑传递信号
        public float currentZNormalizedRot;//转换成 0 -1float信号的 当前旋转角度可能是给大脑传递信号

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;//解算器为满足所有约束(当前认为是关节)而施加的力 可能是被其他关节影响后
                                         //自身的向量力 力方向

        public float currentJointForceSqrMag;//力这个向量的长度 力大小
        public Vector3 currentJointTorque;//解算器为满足所有约束而施加的扭矩(旋转的力) 旋转力大小
        public float currentJointTorqueSqrMag;//这个旋转力向量的长度 旋转力方向
        //public AnimationCurve jointForceCurve = new AnimationCurve();//创造保存某时刻力的时间轴(不确定)
        //public AnimationCurve jointTorqueCurve = new AnimationCurve();//创造保存某时刻旋转力的时间轴(不确定)

        /// <summary>
        /// Reset body part to initial configuration.
        /// </summary>
        public void Reset(BodyPart bp)//重置关节为初始状态 可能用于重置一集训练
        {
            bp.rb.transform.position = bp.startingPos;//当前的位置复原成最初位置
            bp.rb.transform.rotation = bp.startingRot;//当前的旋转复原成最初旋转
            bp.rb.velocity = Vector3.zero;//速度归零
            bp.rb.angularVelocity = Vector3.zero;//角速度归零
            if (bp.groundContact)//如果接触地面信号为 true 重置信号
            {
                bp.groundContact.touchingGround = false; //则为false 
            }

            //if (bp.targetContact)
            //{
            //    bp.targetContact.touchingTarget = false;
            //}
        }

        /// <summary>
        /// Apply torque according to defined goal `x, y, z` angle and force `strength`.
        /// </summary>
        public void SetJointTargetRotation(float x, float y, float z)//接受大脑信号后 进行旋转的角度
        {
            x = (x + 1f) * 0.5f;//初始位置为中点0.5 猜测 x y z 的大小范围均为 [-1,1]
            y = (y + 1f) * 0.5f;//初始位置为中点0.5
            z = (z + 1f) * 0.5f;//初始位置为中点0.5
            //取得值为 (joint.highAngularXLimit.limit -  joint.lowAngularXLimit.limit) * x(0-1之间的值) + joint.lowAngularXLimit.limit
            var xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);//获取最低与最高之间
                                                                                                  //百分之x的值
                                                                                                  //即当 x为 0.5 时
                                                                                                  //取得值为 60-(-60) * 0.5 +(-60) =0
                                                                                                  //将 [0,1]变成角度值[-60,60]

            var yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
            var zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);

            currentXNormalizedRot =
                Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, xRot);//返回0-1之间值
                                                                                                     //将 角度值[-60,60]变成[0,1]
            currentYNormalizedRot = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, yRot);
            currentZNormalizedRot = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, zRot);

            joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);//按欧拉角旋转 设置m_TargetRotation目标旋转方向
            currentEularJointRotation = new Vector3(xRot, yRot, zRot);//保存当前旋转
        }

        public void SetJointStrength(float strength)//设置Joint关节SlerpDrive的力m_SlerpDrive.maximumForce
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;//thisJdController.maxJointForceLimit
                                                                                      //官方配置10000
            var jd = new JointDrive
            {
                positionSpring = thisJdController.maxJointSpring,//弹力 官方配置3000
                positionDamper = thisJdController.jointDampen,//阻力 官方配置30
                maximumForce = rawVal// 通过公式取得当前最大力
            };
            joint.slerpDrive = jd; //赋值给slerpDrive
            currentStrength = jd.maximumForce;//记录当前最大力
        }
    }

    public class JointDriveController : MonoBehaviour
    {
        [Header("Joint Drive Settings")]//实例化显示名字 "JointDrive配置面板" 
        [Space(100)]//名字和下方参数的距离
        public float maxJointSpring;//设置JointDrive的最大关节弹力

        public float jointDampen;//设置JointDrive的阻尼力
        public float maxJointForceLimit;//设置JointDrive最大力极限

        [HideInInspector] public Dictionary<Transform, BodyPart> bodyPartsDict = new Dictionary<Transform, BodyPart>();
                                                                //一个字典 的key是transform value是bodypart  

        [HideInInspector] public List<BodyPart> bodyPartsList = new List<BodyPart>();
                                                                //列表(一维数组) 存每个关节的BodyPart组件
        const float k_MaxAngularVelocity = 50.0f;//最大角速度 maxAngularVelocity	刚体的最大角速率（以弧度/秒为单位）。（默认值为 7）范围为 { 0, 无穷大 }。
        //public AnimationCurve jointTorqueCurve;
        //public AnimationCurve jointForceCurve;
        /// <summary>
        /// Create BodyPart object and add it to dictionary.
        /// </summary>
        public void SetupBodyPart(Transform t) //初始化设置BodyPart
        {
            var bp = new BodyPart 
            {
                rb = t.GetComponent<Rigidbody>(),//初始化赋值此transfrom底下的Rigidbody组件
                joint = t.GetComponent<ConfigurableJoint>(),//初始化赋值此transfrom底下的joint组件
                startingPos = t.position,//BodyPart开始位置
                startingRot = t.rotation//BodyPart开始旋转
            };
            bp.rb.maxAngularVelocity = k_MaxAngularVelocity; //设置rigidbody的最大角速率

            // Add & setup the ground contact script
            bp.groundContact = t.GetComponent<GroundContact>();//初始化赋值此transfrom底下的地面接触组件
            if (!bp.groundContact)//如果此transform底下没有地面接触组件
            {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();//那么添加一个地面接触组件并且初始化赋值
                bp.groundContact.agent = gameObject.GetComponent<Agent>();//初始化赋值
                                                                          //groundContact这个组件中的agent参数
                                                                          //为附带本组件jointDriveController的
                                                                          //游戏对象底下的Agent组件
            }
            else//有了groundContact组件就不用给带有BodyPart的游戏对象添加组件
            {
                bp.groundContact.agent = gameObject.GetComponent<Agent>();//初始化赋值
                                                                          //groundContact这个组件中的agent参数
                                                                          //为附带本组件jointDriveController的
                                                                          //游戏对象底下的Agent组件
            }

            if (bp.joint)//如果有关节
            {
                var jd = new JointDrive//那么关节的参数初始化
                {
                    positionSpring = maxJointSpring,//最大弹力
                    positionDamper = jointDampen,//阻尼力
                    maximumForce = maxJointForceLimit//最大力限制
                };
                bp.joint.slerpDrive = jd;//将此关节的slerpDrive设成初始化的jd
            }

            bp.thisJdController = this;//此BodyPart关节带的参数thisJdController 等于 自身
            bodyPartsDict.Add(t, bp);//保存进字典
            bodyPartsList.Add(bp);//保存进列表
        }

        //public void GetCurrentJointForces()
        //{
        //    foreach (var bodyPart in bodyPartsDict.Values)//遍历所有字典的BodyPart
        //    {
        //        if (bodyPart.joint)//如果有关节
        //        {
        //            bodyPart.currentJointForce = bodyPart.joint.currentForce;//保存currentForce进参数currentJointForce
        //            bodyPart.currentJointForceSqrMag = bodyPart.joint.currentForce.magnitude;//保存currentForce.magnitude进参数currentJointForceSqrMag
        //            bodyPart.currentJointTorque = bodyPart.joint.currentTorque;//保存joint.currentTorque进参数currentJointTorque
        //            bodyPart.currentJointTorqueSqrMag = bodyPart.joint.currentTorque.magnitude;//保存.currentTorque.magnitude进参数currentJointTorqueSqrMag
        //            if (Application.isEditor)//如果是在编辑器状态下运行
        //            {
        //                if (bodyPart.jointForceCurve.length > 1000)//大于1000 保存力的时间轴长度超过1000就新加一个重新保存
        //                {
        //                    bodyPart.jointForceCurve = new AnimationCurve();//AnimationCurve保存可随时间评估的 Keyframes 的集合。
        //
        //
        //                }
        //
        //                if (bodyPart.jointTorqueCurve.length > 1000)//保存旋转力的时间轴长度超过1000就新加一个重新保存
        //                {
        //                    bodyPart.jointTorqueCurve = new AnimationCurve();
        //                }
        //
        //                bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag);//保存当前时间点的力
        //                bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag);//保存当前时间点的旋转力
        //            }
        //        }
        //    }
        //}
    }
}
