# 是否需要颜色限制？需要填1，不需要填0
choice = 1

# 导入opencv库
import cv2
import numpy as np

# 创建一个VideoCapture对象，从摄像头获取图像
cap = cv2.VideoCapture(0)

# 创建一个窗口，显示摄像头画面
cv2.namedWindow("Camera")

# 循环处理每一帧图像
while True:
    # 读取一帧图像
    ret, frame = cap.read()
    if not ret:
        break

    # 获取图像的宽度和高度
    height, width = frame.shape[:2]

    # 颜色限制部分
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰色通道
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)  # 转换为HSV空间
    lower_green = np.array([60,50,50])  # 设定绿色的阈值下限
    upper_green = np.array([130,255,255])  # 设定绿色的阈值上限
    mask = cv2.inRange(hsv, lower_green, upper_green)  # 设定掩膜取值范围
    kernel = np.ones((5, 5), np.uint8)  # 卷积核
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 形态学开运算
    bila = cv2.bilateralFilter(opening, 10, 200, 200)  # 双边滤波消除噪声
    Direct = cv2.bilateralFilter(gray, 10, 200, 200)
    # 使用霍夫变换检测圆形，返回一个数组，每个元素是一个圆的参数（x, y, r）
     
    if choice == 1:
        # 需要颜色限制
        circles = cv2.HoughCircles(bila, cv2.HOUGH_GRADIENT_ALT, 1, 100, param1=100, param2=0.7, minRadius=15, maxRadius=200)
    else:   
        # 不需要颜色限制
        circles = cv2.HoughCircles(Direct, cv2.HOUGH_GRADIENT_ALT, 1, 100, param1=100, param2=0.7, minRadius=15, maxRadius=200)


    # 如果检测到圆形
    if circles is not None:
        circles = sorted(circles[0], key=lambda c: c[2], reverse=True)

        # 取第一个元素作为最大的圆
        x, y, r = circles[0]

        # 在图像上绘制呼啦圈的边界和中心
        cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
        cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), 3)

        # 计算呼啦圈中心与图像中心的偏差
        dx = x - width // 2
        dy = y - height // 2

        # 设置一个阈值，如果偏差小于阈值，则认为已经对准
        threshold = 10

        # 根据偏差输出相应的指令
        if abs(dx) < threshold and abs(dy) < threshold:
            text = "Centered"
        else:
            if dx > threshold:
                text = "Move right"
            elif dx < -threshold:
                text = "Move left"
            if dy > threshold:
                text = "Move down"
            elif dy < -threshold:
                text = "Move up"

        # 在图像上显示提示信息
        cv2.putText(frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    # 显示图像
    cv2.imshow("Camera", frame)
    cv2.imshow("mask",mask)

    # 等待按键，如果按下q键，则退出循环
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# 释放资源并关闭窗口
cap.release()
cv2.destroyAllWindows()