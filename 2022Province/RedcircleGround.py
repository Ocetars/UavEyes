import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("D:\\Gitworkspace\\UavEyes\\2022Province\\RedCircleGround.mp4")
# 创建具有指定大小的窗口
cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Frame", 1280, 720)
cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Mask", 1280, 720)
while True:
    # 从摄像头捕获图像
    ret, frame = cap.read()
    height, width = frame.shape[:2]
    
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
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=4)
    # 在掩码中查找圆形
    # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT_ALT, 1, 100, param1=100, param2=0.7, minRadius=15, maxRadius=2000)

    # # 如果检测到圆形
    # if circles is not None:
    #     # 获取最大轮廓
    #     circles = sorted(circles[0], key=lambda c: c[2], reverse=True)
    #     # 取第一个元素作为最大的圆
    #     x, y, r = circles[0]
    #     # 将x和y转换为整数类型
    #     x, y, r = int(x), int(y), int(r)
    #     cv2.circle(frame, (x, y), 2, (0, 255, 0), 3)
    #     # 在圆周上绘制一个圆
    #     cv2.circle(frame, (x, y), r, (0, 0, 255), 2)
    #     # 计算呼啦圈中心与图像中心的偏差
    #     dx = x - width // 2
    #     dy = y - height // 2
    #     # 设置一个阈值，如果偏差小于阈值，则认为已经对准
    #     threshold = 10
    #     # 根据偏差输出相应的指令
    #     direction = "Unknown"
    #     if abs(dx) < threshold and abs(dy) < threshold:
    #         direction = "Centered"
    #     else:
    #         if dx > threshold:
    #             direction = "Move right"
    #         elif dx < -threshold:
    #             direction = "Move left"
    #         if dy > threshold:
    #             direction = "Move down"
    #         elif dy < -threshold:
    #             direction = "Move up"
                
    #     print(direction)

    # # 显示图像
    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

    # 按'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
