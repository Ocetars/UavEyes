import cv2
import numpy as np


# 定义一个函数，用于计算直线和画面中心的距离
def distance_to_center(line):
    # 获取直线的端点坐标
    x1, y1, x2, y2 = line[0]
    # 获取画面的中心坐标
    cx, cy = (
        cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2,
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2,
    )
    # 计算直线的向量
    vx, vy = x2 - x1, y2 - y1
    # 计算中心点到直线的向量
    wx, wy = cx - x1, cy - y1
    # 计算向量的叉积
    cross = abs(vx * wy - vy * wx)
    # 计算直线的长度
    length = np.sqrt(vx**2 + vy**2)
    # 返回点到直线的距离
    return cross / length


# 定义一个函数，用于检测最靠近中心的直线
def detect_line(
    image, rho=1, theta=np.pi / 180, threshold=150, minLineLength=50, MaxValue=1000
):
    # 转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 检测边缘
    edges = cv2.Canny(gray, 100, 250, apertureSize=3, L2gradient=True)
    cv2.imshow("edges", edges)
    # 检测直线
    lines = cv2.HoughLinesP(
        edges, rho, theta, threshold, minLineLength, maxLineGap=MaxValue
    )
    # 如果检测到直线，返回最靠近中心的直线的端点坐标和距离
    if lines is not None:
        # 初始化最小距离和最小距离对应的直线索引
        min_dist = float("inf")
        min_index = -1
        # 遍历所有的直线，计算距离，并更新最小值和索引
        for i in range(len(lines)):
            line = lines[i]
            dist = distance_to_center(line)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        # 返回最小距离对应的直线的端点坐标和距离
        x1, y1, x2, y2 = lines[min_index][0]

        return (x1, y1), (x2, y2), min_dist
    else:
        # 如果没有检测到直线，返回None
        return None


"""
以下是主程序，仅用于测试函数
"""

# 创建视频捕捉对象
cap = cv2.VideoCapture(0)

# 循环处理每一帧
while True:
    # 读取一帧
    ret, frame = cap.read()
    # 如果读取失败，退出循环
    if not ret:
        break
    # 调用detect_line函数，检测最靠近中心的直线
    result = detect_line(frame)
    # 如果检测到直线，绘制在原图上，并打印端点坐标和距离值
    if result is not None:
        (x1, y1), (x2, y2), min_dist = result
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        print(f"({x1}, {y1}), ({x2}, {y2}), {min_dist}")
    elif result is None:
        print("line none")
    else:
        print("error")

    cv2.imshow("Frame", frame)
    # 等待按键
    key = cv2.waitKey(1)
    # 如果按下q键，退出循环
    if key == ord("q"):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()