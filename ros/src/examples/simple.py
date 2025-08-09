#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Empty
import time

def simple_driver():
    """
    가장 기본적인 차량 조작을 위한 예제 노드
    1. 10초간 풀 스로틀(1.0)과 최대 우회전(0.5) 명령을 보냄
    2. 10초 후, 풀 브레이크(1.0) 명령을 보냄
    """
    rospy.init_node('simple_driver_node')

    # 제어 명령을 보낼 Publisher 생성
    control_pub = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)
    
    # 주행 시작 신호를 보낼 Publisher 생성
    go_pub = rospy.Publisher('/fsds/signal/go', Empty, queue_size=1)

    # ROS 시스템이 준비될 때까지 잠시 대기
    rospy.sleep(1.0)

    # 'Go' 신호 발행하여 주행 시작
    go_pub.publish(Empty())
    rospy.loginfo("주행 시작 'Go' 신호를 발행했습니다")
    rospy.sleep(1.0)

    # 제어 명령 메시지 생성
    drive_cmd = ControlCommand()
    drive_cmd.throttle = 1.0  # 풀 스로틀
    drive_cmd.steering = 0.5  # 최대 우회전 (양수 값이 우회전)

    rospy.loginfo("10초간 풀 스로틀 및 우회전을 시작합니다")

    # 시작 시간 기록
    start_time = time.time()
    
    # 10초 동안 주행 명령을 계속 보냄 (50Hz, 즉 0.02초 간격)
    rate = rospy.Rate(50) 
    while time.time() - start_time < 10.0:
        control_pub.publish(drive_cmd)
        rate.sleep()

    rospy.loginfo("10초 경과. 풀 브레이크를 시작합니다.")

    # 정지 명령 메시지 생성
    stop_cmd = ControlCommand()
    stop_cmd.throttle = 0.0
    stop_cmd.steering = 0.0 # 스티어링은 중앙으로
    stop_cmd.brake = 1.0    # 풀 브레이크

    # 차량이 완전히 멈출 때까지 2초간 정지 명령을 보냄
    stop_time = time.time()
    while time.time() - stop_time < 2.0:
        control_pub.publish(stop_cmd)
        rate.sleep()

    rospy.loginfo("주행이 종료되었습니다.")


if __name__ == '__main__':
    try:
        simple_driver()
    except rospy.ROSInterruptException:
        pass