import sys
sys.path.append("D:\\Gitworkspace\\UavEyes")
import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("D:\\Gitworkspace\\UavEyes\\2022Province\\blackCircle.mp4")
# 创建具有指定大小的窗口
cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Frame", 1280, 720)
while True:
    # 从摄像头捕获图像
    ret, frame = cap.read()
    height, width = frame.shape[:2]
    
    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义红色范围
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 140])
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # 对掩码进行形态学操作以消除噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=4)
    # 在掩码中查找圆圈
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 如果检测到圆形
    if contours:
        # 获取最大轮廓
        contour = max(contours, key=cv2.contourArea)
        # 如果轮廓点数足够，拟合椭圆
        if len(contour) >= 5:
            # 拟合椭圆
            ellipse = cv2.fitEllipse(contour)
            # 获取椭圆参数
            center, axes, angle = ellipse
            x1, y1 = center
            # 绘制椭圆和识别框
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
            # cv2.rectangle(frame, (int(center[0] - axes[0] / 2), 
            #                     int(center[1] - axes[1] / 2)), 
            #                     (int(center[0] + axes[0] / 2), 
            #                     int(center[1] + axes[1] / 2)), (0, 255, 0), 2)
            # print("Ellipse detected at: ({}, {})".format(int(center[0]), int(center[1])))
            # 计算呼啦圈中心与图像中心的偏差
            dx = x1 - width // 2
            dy = y1 - height // 2

            # 设置一个阈值，如果偏差小于阈值，则认为已经对准
            threshold = 10

            # 根据偏差输出相应的指令
            direction = "Unknown"
            if abs(dx) < threshold and abs(dy) < threshold:
                direction = "Centered"
            else:
                if dx > threshold:
                    direction = "Move right"
                elif dx < -threshold:
                    direction = "Move left"
                if dy > threshold:
                    direction = "Move down"
                elif dy < -threshold:
                    direction = "Move up"
                
        print(direction)

    # 显示图像
    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

    # 按'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
