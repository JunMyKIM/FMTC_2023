#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin

# 그래프 설정
fig, ax = plt.subplots()
xdata, ydata, arrows = [], [], []

def update_plot(data):
    xdata.append(data.x)
    ydata.append(data.y)

    # 데이터 범위에 따른 화살표 길이 계산
    data_range = max(max(xdata) - min(xdata), max(ydata) - min(ydata), 1)
    arrow_length = data_range * 0.05  # 화살표 길이를 데이터 범위의 5%로 설정

    dx = arrow_length * cos(data.theta)
    dy = arrow_length * sin(data.theta)

    # 기존 화살표 제거
    for arrow in arrows:
        arrow.remove()
    arrows.clear()

    # 새로운 화살표 추가
    arrow = ax.arrow(data.x, data.y, dx, dy, head_width=arrow_length*0.1, head_length=arrow_length*0.2, fc='red', ec='red')
    arrows.append(arrow)

    # 궤적을 점으로 표현
    ax.scatter(xdata, ydata, color='blue', s=10)  # s는 점의 크기를 설정
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Vehicle Trajectory and Heading')

def listener():
    rospy.init_node('trajectory_plotter', anonymous=True)
    rospy.Subscriber("vehicle_pose", Pose2D, update_plot)
    ani = animation.FuncAnimation(fig, lambda frame: None, interval=100)
    plt.show()

if __name__ == '__main__':
    listener()
