import cv2
import numpy as np

cap = cv2.VideoCapture(1)
while True:
    # 读取摄像头画面
    ret, frame = cap.read()

    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)

    # 定义绿色和红色的HSV阈值范围
    lower_green = np.array([70, 50, 50])
    upper_green = np.array([130, 255, 255])
    
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([20, 255, 255])
    lower_red2 = np.array([330, 50, 50])
    upper_red2 = np.array([360, 255, 255])

    # 提取绿色和红色杆子的区域
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red, upper_red)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 对二值图像进行形态学操作，去除干扰的白色网格
    kernel = np.ones((5, 5), np.uint8)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    # 对二值图像进行腐蚀操作
    kernel = np.ones((5, 5), np.uint8)
    # mask_green = cv2.erode(mask_green, kernel, iterations=int(0.2))
    mask_red = cv2.erode(mask_red, kernel, iterations=5)

    # 对二值图像进行膨胀操作
    kernel = np.ones((5, 5), np.uint8)
    mask_green = cv2.dilate(mask_green, kernel, iterations=1)
    mask_red = cv2.dilate(mask_red, kernel, iterations=3)
    
    # 对二值图像进行形态学操作，填充轮廓内部的空洞
    kernel = np.ones((7, 7), np.uint8)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    # 查找绿色和红色杆子的轮廓
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 绘制绿色和红色杆子的轮廓
    for contour in contours_green:
        if cv2.contourArea(contour) > 3000 and cv2.contourArea(contour) < 16000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    for contour in contours_red:
        if cv2.contourArea(contour) > 3000 and cv2.contourArea(contour) < 16000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # 显示图像和膨胀后的图像
    cv2.imshow("image", frame)
    cv2.imshow("dilated_green", mask_green)
    cv2.imshow("dilated_red", mask_red)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.waitKey(0)
cv2.destroyAllWindows()