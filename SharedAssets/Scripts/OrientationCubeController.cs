using UnityEngine;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Utility class to allow a stable observation platform.
    /// </summary>
    public class OrientationCubeController : MonoBehaviour
    {
        //Update position and Rotation
        public void UpdateOrientation(Transform rootBP, Transform target)//更新方块的方向
                                                                         //这个方块始终会在BodySeg0的位置
                                                                         //并始终指向绿色方块
        {
            var dirVector = target.position - transform.position;//将两个位置之间差距的向量
            dirVector.y = 0; //flatten dir on the y. this will only work on level, uneven surfaces
                             //不考虑高度差 只在平面位置上进行寻路
            
            var lookRot =
                dirVector == Vector3.zero
                    ? Quaternion.identity //旋转X,Y,Z都为0,如果目标和自身在同一个位置设置lookRot为(0,0,0,1)
                    : Quaternion.LookRotation(dirVector); //get our look rot to the target
                                                          //如果不为同一个位置,
                                                          //则lookRot等于将自身旋转面向目标的Rotation
            
            //UPDATE ORIENTATION CUBE POS & ROT
            transform.SetPositionAndRotation(rootBP.position, lookRot);//自身的位置设置成当前BodySeg0的位置
                                                                       //并面向目标
        }
    }
}
