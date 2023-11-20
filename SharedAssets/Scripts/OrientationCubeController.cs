using UnityEngine;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Utility class to allow a stable observation platform.
    /// </summary>
    public class OrientationCubeController : MonoBehaviour
    {
        //Update position and Rotation
        public void UpdateOrientation(Transform rootBP, Transform target)//���·���ķ���
                                                                         //�������ʼ�ջ���BodySeg0��λ��
                                                                         //��ʼ��ָ����ɫ����
        {
            var dirVector = target.position - transform.position;//������λ��֮���������
            dirVector.y = 0; //flatten dir on the y. this will only work on level, uneven surfaces
                             //�����Ǹ߶Ȳ� ֻ��ƽ��λ���Ͻ���Ѱ·
            
            var lookRot =
                dirVector == Vector3.zero
                    ? Quaternion.identity //��תX,Y,Z��Ϊ0,���Ŀ���������ͬһ��λ������lookRotΪ(0,0,0,1)
                    : Quaternion.LookRotation(dirVector); //get our look rot to the target
                                                          //�����Ϊͬһ��λ��,
                                                          //��lookRot���ڽ�������ת����Ŀ���Rotation
            
            //UPDATE ORIENTATION CUBE POS & ROT
            transform.SetPositionAndRotation(rootBP.position, lookRot);//�����λ�����óɵ�ǰBodySeg0��λ��
                                                                       //������Ŀ��
        }
    }
}
