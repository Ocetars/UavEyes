import cv2
import numpy as np


# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("D:\\Gitworkspace\\UavEyes\\2021\\oak.mp4")

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

center_x = int(width / 2)
center_y = int(height / 2)

# 定义绿色范围
lower_green = np.array([60, 30, 50])
upper_green = np.array([90, 255, 255])

# 定义中心区域大小
region_size = 35

while True:
    ret, frame = cap.read()
    # 将帧转换为HSV颜色空间
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    # 提取中心区域像素
    center_region = mask[
        center_y - region_size // 2 : center_y + region_size // 2,
        center_x - region_size // 2 : center_x + region_size // 2,
    ]
    # 计算中心区域像素平均值
    avg_pixel = cv2.mean(center_region)
    print(avg_pixel)
    # 判断中心区域像素平均值是否在绿色范围内
    if avg_pixel[0] > 180:
        print(1)
        result = 1
    else:
        print(0)
        result = 0
    # 在图像上绘制中心检测区域
    cv2.rectangle(
        frame,
        (center_x - region_size // 2, center_y - region_size // 2),
        (center_x + region_size // 2, center_y + region_size // 2),
        (0, 255, 0),
        2,
    )
    # 在图像上显示结果
    cv2.putText(
        frame, str(result), (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 2
    )

    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    if cv2.waitKey(25) & 0xFF == ord("q"):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
