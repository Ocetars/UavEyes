import cv2
import numpy as np
import math
import rospy
from geometry_msgs.msg import Point

rospy.init_node('redcircle_ground_publisher')
pub = rospy.Publisher('redcircle_ground', Point, queue_size=10)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap = cv2.VideoCapture('src/RedCircleGround.mp4')

while not rospy.is_shutdown():
    # 从摄像头捕获图像
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    # height, width = frame.shape[:2]
    
    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义红色范围
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # 将两个掩码组合在一起
    mask = mask1 + mask2

    # 对掩码进行形态学操作以消除噪声
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=4)
    # 查找图像中的所有轮廓
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 循环遍历所有轮廓
    for contour in contours:
        # 计算轮廓的面积和周长
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        # 如果周长为零，则跳过此轮廓
        if perimeter == 0:
            continue
        # 计算近似轮廓并计算其与原始轮廓之间的差异
        approx = cv2.approxPolyDP(contour, 0.01 * perimeter, True)
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        # 如果轮廓是圆形，则绘制它
        if len(approx) > 8 and circularity > 0.7 and area > 1000:
            # cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            ellipse = cv2.fitEllipse(contour)
            # 获取椭圆参数
            center, axes, angle = ellipse
            major_axis, minor_axis = axes
            x1, y1 = center
            x_center, y_center = int(x1), int(y1)
            
            # 在图像上绘制中心点
            cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
            # 在图像上绘制中心点的值
            cv2.putText(frame, f"({x_center}, {y_center})", (x_center, y_center), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
            # 绘制椭圆和识别框
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
            # Create a Point message and set its x and y fields
            point_msg = Point()
            point_msg.x = x_center
            point_msg.y = y_center
            rospy.loginfo(point_msg)
            # Publish the message to the "redcircle_ground" topic
            pub.publish(point_msg)
            # print("Ellipse detected at: ({}, {})".format(x_center, y_center))

    # 显示图像
    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

    # 按'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
