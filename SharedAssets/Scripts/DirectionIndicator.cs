using UnityEngine;

namespace Unity.MLAgentsExamples
{
    public class DirectionIndicator : MonoBehaviour
    {

        //public bool updatedByAgent; //should this be updated by the agent? If not, it will use local settings
        //public Transform transformToFollow; //ex: hips or body
        //public Transform targetToLookAt; //target in the scene the indicator will point to
        public float heightOffset;
        private float m_StartingYPos;

        void OnEnable()
        {
            m_StartingYPos = transform.position.y;
        }

        //void Update()
        //{
        //    if (updatedByAgent)//如果通过Agent进行Update更新的话就 return
        //        return; //不运行后面代码 直接返回空
        //    transform.position = new Vector3(transformToFollow.position.x, m_StartingYPos + heightOffset,
        //        transformToFollow.position.z);//自身位置等于BodySeg0 可以设置高度偏差heightOffset
        //    Vector3 walkDir = targetToLookAt.position - transform.position;//将两个位置之间差距的向量(包括方向和路程)
        //
        //    walkDir.y = 0; //flatten dir on the y 平面运算不考虑高度差
        //    transform.rotation = Quaternion.LookRotation(walkDir);//自身的面转向 指向绿色方块的方向
        //}

        //Public method to allow an agent to directly update this component
        public void MatchOrientation(Transform t)//直接设置成方块的位置和方向
        {
            transform.position = new Vector3(t.position.x, m_StartingYPos + heightOffset, t.position.z);
            transform.rotation = t.rotation;
        }
    }
}
