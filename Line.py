"""
主要函数说明
HoughLinesP(image, rho, theta, threshold, lines=None, minLineLength=None, maxLineGap=None) 
其中，参数的含义如下：
image: 必须是二值图像,推荐使用canny边缘检测的结果图像
rho: 线段以像素为单位的距离精度,double类型的,推荐用1.0 
theta: 线段以弧度为单位的角度精度,推荐用numpy.pi/180 
threshod: 累加平面的阈值参数,int类型,超过设定阈值才被检测出线段,值越大,基本上意味着检出的线段越长,检出的线段个数越少。根据情况推荐先用100试试
minLineLength: 线段以像素为单位的最小长度,根据应用场景设置 
maxLineGap: 最大直线间隙，如果两条直线之间的间隙小于该值，则认为它们是一条直线。

cv2.canny的用法是:
edges = cv2.Canny(image, threshold1, threshold2, apertureSize, L2gradient)
其中，参数的含义如下：
image: 输入的灰度图像，可以是边缘检测前的图像。
threshold1: 第一个阈值，用于检测边缘的强度。
threshold2: 第二个阈值，用于检测边缘的连接性。
apertureSize: Sobel算子的核大小,用于计算图像的梯度。默认值是3。
L2gradient: 是否使用更精确的L2范数来计算梯度强度。默认值是False。
"""

import cv2
import numpy as np

# 定义一个函数，用于计算直线和画面中心的距离
def distance_to_center(line):
    # 获取直线的端点坐标
    x1, y1, x2, y2 = line[0]
    # 获取画面的中心坐标
    cx, cy = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2, cap.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2
    # 计算直线的向量
    vx, vy = x2 - x1, y2 - y1
    # 计算中心点到直线的向量
    wx, wy = cx - x1, cy - y1
    # 计算向量的叉积
    cross = abs(vx * wy - vy * wx)
    # 计算直线的长度
    length = np.sqrt(vx ** 2 + vy ** 2)
    # 返回点到直线的距离
    return cross / length
    """
    叉积的定义是，两个向量的乘积等于它们所构成的平行四边形的面积。
    如果一个向量是从直线的一个端点指向另一个端点，另一个向量是从直线的一个端点指向画面中心，
    那么它们的叉积就等于这两个向量所构成的三角形的面积。而这个三角形的高，就是点到直线的距离。
    所以，我们只需要用叉积除以直线的长度，就可以得到距离。
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
    # 转换为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 检测边缘
    edges = cv2.Canny(gray, 100, 250, apertureSize=3, L2gradient=True)
    cv2.imshow("edges", edges)
    # 检测直线
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 150, minLineLength=50, maxLineGap=1000)
    # 如果检测到直线，绘制在原图上
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
        # 只绘制最小距离对应的直线，而忽略其他的直线
        x1, y1, x2, y2 = lines[min_index][0]
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 打印端点坐标和距离值
        print(f"({x1}, {y1}), ({x2}, {y2}), {min_dist}")
    # 显示结果
    cv2.imshow("Frame", frame)
    # 等待按键
    key = cv2.waitKey(1)
    # 如果按下q键，退出循环
    if key == ord("q"):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()