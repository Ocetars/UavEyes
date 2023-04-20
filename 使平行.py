import cv2
import numpy as np

# 创建一个窗口来显示图像和指示
cv2.namedWindow ("Image")

# 打开摄像头
cap = cv2.VideoCapture (0)

# 循环处理每一帧图像
while True:
    # 读取一帧图像
    ret, frame = cap.read ()
    if not ret:
        break
    
    # 转换为灰度图像
    gray = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)

    # 边缘检测
    edges = cv2.Canny(gray, 50, 150)

    # 轮廓检测
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    # 如果找到了轮廓
    if contours:
        # 取最大的轮廓
        cnt = max (contours, key=cv2.contourArea)
        # 检查轮廓的点数是否大于等于5
        if len(cnt) >= 5:
            # 拟合椭圆
            ellipse = cv2.fitEllipse(cnt)
            # 绘制椭圆
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
            # 获取椭圆参数
            (x, y), (a, b), angle = ellipse
            # 根据角度给出提示信息
            if angle < 90:
                text = "Rotate clockwise by {:.1f} degrees".format(90 - angle)
            elif angle > 90:
                text = "Rotate counterclockwise by {:.1f} degrees".format(angle - 90)
            else:
                text = "No need to rotate"
            # 在图像上显示提示信息
            cv2.putText(frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            # 删除轮廓
            cv2.drawContours(frame, [cnt], -1, (0, 0, 0), -1)
        # 在图像上显示提示信息
        cv2.putText(frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 显示图像和指示
        cv2.imshow ("Image", frame)
        cv2.imshow ("edge", gray)

            
    # 按ESC键退出循环
    key = cv2.waitKey (1) & 0xFF
    if key == 27:
        break

# 关闭摄像头和窗口
cap.release ()
cv2.destroyAllWindows ()