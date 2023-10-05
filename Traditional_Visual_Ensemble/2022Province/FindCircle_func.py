import cv2
import numpy as np

def find_circle(frame, 
                choice=0, 
                lower_color=np.array([60,50,50]), 
                upper_color=np.array([130,255,255])):
    # 获取图像的宽度和高度
    height, width = frame.shape[:2]

    # 颜色限制部分
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰色通道
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)  # 转换为HSV_FULL空间
    mask = cv2.inRange(hsv, lower_color, upper_color)  # 设定掩膜取值范围
    kernel = np.ones((5, 5), np.uint8)  # 卷积核
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 形态学开运算
    bila = cv2.bilateralFilter(opening, 10, 200, 200)  # 双边滤波消除噪声
    Direct = cv2.bilateralFilter(gray, 10, 200, 200)

    # 使用霍夫变换检测圆形，返回一个数组，每个元素是一个圆的参数（x, y, r）
    if choice == 1:
        # 需要颜色限制
        circles = cv2.HoughCircles(bila, cv2.HOUGH_GRADIENT_ALT, 1, 100, param1=100, param2=0.7, minRadius=15, maxRadius=200)
    elif choice == 0:
        # 不需要颜色限制
        circles = cv2.HoughCircles(Direct, cv2.HOUGH_GRADIENT_ALT, 1, 100, param1=100, param2=0.7, minRadius=15, maxRadius=200)

    # 如果检测到圆形
    if circles is not None:
        circles = sorted(circles[0], key=lambda c: c[2], reverse=True)
        # 取第一个元素作为最大的圆
        x, y, r = circles[0]
        cv2.circle(frame, (x, y), 2, (0, 255, 0), 3)
        # 在圆周上绘制一个圆
        cv2.circle(frame, (x, y), r, (0, 0, 255), 2)
        # 计算呼啦圈中心与图像中心的偏差
        dx = x - width // 2
        dy = y - height // 2

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

        # 返回圆心坐标和最后判断出的方向
        return (x, y), direction, frame, bila

    # 如果没有检测到圆形，则返回空值
    return None, None ,frame, bila