#!/usr/bin/env python3
import rospy
import random
import time
from std_msgs.msg import Int32

def publish_virtual_diagnostic():
    rospy.init_node('virtual_diagnostic_node', anonymous=True)
    pub = rospy.Publisher('virtual_diagnostic', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    end_time = rospy.Time.now() + rospy.Duration(60)  # 1분 동안 실행
    while rospy.Time.now() < end_time:
        # 대부분의 시간 동안 '1' 발행
        pub.publish(1)

        # 랜덤하게 '2' 발행
        if random.random() < 0.05:  # 약 5% 확률로 '2'를 발행
            for _ in range(3):
                pub.publish(2)
                rospy.sleep(3)  # 3초 동안 대기

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_virtual_diagnostic()
    except rospy.ROSInterruptException:
        pass
