import cv2
import numpy as np
import math

def detect_closest_line(image, rho=1, theta=np.pi / 180, threshold=150, minLineLength=50, MaxValue=1000):
    # 转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 检测边缘
    edges = cv2.Canny(gray, 100, 250, apertureSize=3, L2gradient=True)
    cv2.imshow("edges", edges)
    # 检测直线
    lines = cv2.HoughLinesP(
        edges,
        rho,
        theta,
        threshold,
        minLineLength,
        maxLineGap=MaxValue,
    )
    # 如果没有检测到直线，返回 None
    if lines is None:
        return None
    # 获取最靠近中心的直线
    closest_line = None
    closest_distance = float("inf")
    for line in lines:
        x1, y1, x2, y2 = line[0]
        distance = abs((y2 - y1) * image.shape[1] - (x2 - x1) * image.shape[0] + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
        if distance < closest_distance:
            closest_line = line
            closest_distance = distance
    # 如果没有检测到直线，返回 None
    if closest_line is None:
        return None
    # 获取最小距离对应的直线的端点坐标和距离
    x1, y1, x2, y2 = closest_line[0]
    distance = closest_distance
    return (x1, y1, x2, y2, distance)

"""主要函数参数设置：

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
L2gradient: 是否使用更精确的L2范数来计算梯度强度。默认值是False。"""

# 主函数示例：
if __name__ == "__main__":
    import cv2
    from line_detector import detect_closest_line

    cap = cv2.VideoCapture(1)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("frame", width, height)
    
    while True:
        ret, frame = cap.read()
        # 检测最靠近中心的直线
        result = detect_closest_line(frame)
        # 如果没有检测到直线，跳过本次循环
        if result is None:
            cv2.imshow("frame", frame) 
        else:
            # 在图像上绘制直线
            x1, y1, x2, y2, distance = result
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            # 显示图像
            cv2.imshow("frame", frame)
            print(f"({x1}, {y1}), ({x2}, {y2}), {distance}")
            
        # 按下 q 键退出循环
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # 释放摄像头并关闭窗口
    cap.release()
    cv2.destroyAllWindows()