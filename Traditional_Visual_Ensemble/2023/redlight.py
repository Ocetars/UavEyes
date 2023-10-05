import cv2
import numpy as np
# import rospy
# from geometry_msgs.msg import Point
# from std_msgs.msg import String
import time

# rospy.init_node('redlight_publisher')
# pub = rospy.Publisher('fire_point', Point, queue_size=10)
# Define a variable to store the last alignment time
last_alignment_time = None
# Define a variable to store the alignment threshold
alignment_threshold = 10
# Define a variable to store the alignment duration
alignment_duration = 4

# 打开摄像头
cap = cv2.VideoCapture(1)
# 设置摄像头分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 900)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
# 降低曝光
cap.set(cv2.CAP_PROP_EXPOSURE, -10
        )
while True:
    # 读取摄像头中的图像
    ret, frame = cap.read()
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2    
    # 将图像从BGR转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义红色范围
    lower_red = np.array([0, 100, 50])
    upper_red = np.array([9, 240, 255])

    # 创建掩膜
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    # 定义红色范围
    lower_red = np.array([170, 100, 46])
    upper_red = np.array([180, 255, 255])

    # 创建掩膜
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # 将两个掩膜相加
    mask = mask1 + mask2
    # 对掩膜进行形态学转换
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    # 在掩码中查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 如果检测到轮廓
    if contours:
        # 获取最大轮廓
        contour = max(contours, key=cv2.contourArea)
        # 检查轮廓中是否至少有5个点，面积大于规定值
        if len(contour) >= 5 and cv2.contourArea(contour) >= 100:
            # 拟合椭圆
            ellipse = cv2.fitEllipse(contour)
            # 获取椭圆参数
            center, axes, angle = ellipse
            x1, y1 = center
            x_center, y_center = int(x1), int(y1)
            
            # 在图像上绘制中心点
            cv2.circle(frame, (x_center, y_center), 5, (0, 255, 0), -1)
            # 在图像上绘制中心点的值
            cv2.putText(frame, f"({x_center}, {y_center})", (x_center, y_center), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
            # 绘制椭圆和识别框
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
            # # Create a Point message and set its x and y fields
            # point_msg = Point()
            # point_msg.x = x_center
            # point_msg.y = y_center
            # rospy.loginfo(point_msg)
            # # Publish the message to the "redcircle_ground" topic
            # pub.publish(point_msg)
        # # Check if the center point is aligned with the ellipse center
        # if abs(x_center - frame_center_x) < alignment_threshold and abs(y_center - frame_center_y) < alignment_threshold:
        #     # If the last alignment time is not set, set it to the current time
        #     if last_alignment_time is None:
        #         last_alignment_time = time.time()
        #     # If the last alignment time is set, check if the alignment duration has been exceeded
            # elif time.time() - last_alignment_time > alignment_duration:
                # Publish a string message to the ROS topic
                # pub = rospy.Publisher('drop_topic', String, queue_size=10)
                # pub.publish("drop")
        # else:
        #     # Reset the last alignment time
        #     last_alignment_time = None
        
    # 显示图像
    cv2.imshow('mask', mask)
    cv2.imshow('frame', frame)

    # 按下q键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()