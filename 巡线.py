# 导入opencv库和数学库
import cv2
import math
import time

# 创建一个视频捕捉对象，参数为摄像头编号，0为默认摄像头
cap = cv2.VideoCapture(0)

# 创建一个窗口，用于显示视频画面
cv2.namedWindow("Video")

# 定义一个函数，计算直线与水平方向的夹角，参数为直线的斜率
def angle(k):
    # 如果斜率接近0，返回0度
    if abs(k) < 0.01:
        return 0
    # 否则，根据反正切函数计算夹角，单位为度
    else:
        return math.degrees(math.atan(k))


# 定义一个函数，计算直线的长度，参数为直线的起点和终点坐标
def length(x1, y1, x2, y2):
    # 根据勾股定理计算直线的长度
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5


# 定义一个函数，判断一个像素是否为红色，参数为像素的BGR值
def is_red(b, g, r):
    # 如果红色分量大于蓝色分量和绿色分量的和，且大于某个阈值（例如100），认为是红色
    return r > b + g and r > 100


# 定义一个函数，计算两点之间的距离，参数为两点的坐标
def distance(x1, y1, x2, y2):
    # 根据勾股定理计算距离
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

# 定义一个函数，计算一个点到一条直线的距离，参数为点的坐标和直线的斜率和截距
def point_to_line(x, y, k, b):
    # 根据点到直线距离公式计算距离
    return abs(k * x - y + b) / (k**2 + 1)**0.5

# 循环处理每一帧画面
while True:
    # 读取一帧画面，ret为读取成功与否的标志，frame为画面数据
    ret, frame = cap.read()
    # 如果读取失败，退出循环
    if not ret:
        break
    
    # 转换画面为灰度图像
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 使用Canny算法检测边缘
    edges = cv2.Canny(gray, 50, 150)
    # 使用Hough变换检测直线，返回值为直线参数的数组
    lines = cv2.HoughLinesP(edges, 1, math.pi/180, 100, minLineLength=100, maxLineGap=10)
    
       # 如果检测到直线，进行处理
    if lines is not None:
        # 初始化最宽的直线为None，记录它的起点和终点坐标，斜率，截距和宽度
        x1, y1, x2, y2 = None, None, None, None
        k = None
        b = None
        w = 0
        # 遍历每一条直线
        for line in lines:
            # 获取直线的起点和终点坐标
            x3, y3, x4, y4 = line[0]
            # 计算直线的中点坐标
            mx = (x3 + x4) // 2
            my = (y3 + y4) // 2
            # 计算直线的宽度，即直线中点到边缘的最大距离
            w2 = 0
            # 遍历直线中点周围的像素，如果是红色，更新宽度
            for i in range(-10, 11):
                for j in range(-10, 11):
                    x = mx + i
                    y = my + j
                    # 如果像素坐标在画面范围内，才进行处理
                    if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                        b, g, r = frame[y][x]
                        if is_red(b, g, r):
                            w2 = max(w2, distance(mx, my, x, y))
            # 如果宽度大于当前最宽的直线的宽度，更新最宽的直线为这条直线，记录它的起点和终点坐标，斜率，截距和宽度
            if w2 > w:
                x1, y1, x2, y2 = x3, y3, x4, y4
                k = (y2 - y1) / (x2 - x1)
                b = y1 - k * x1
                w = w2
        # 如果找到了最宽的直线，进行处理
        if x1 is not None:
            # 计算画面中心点的坐标
            cx = frame.shape[1] // 2
            cy = frame.shape[0] // 2
            # 计算画面中心点到直线的距离
            d = point_to_line(cx, cy, k, b)
            # 如果距离小于10像素，认为直线已经在画面中心位置，不需要移动摄像头
            if d < 10:
                print("No need to move the camera.")
            # 否则，根据直线的斜率和截距，判断应该往哪个方向移动摄像头
            else:
                # 如果截距大于画面中心点的纵坐标，说明直线在画面上半部分，应该向下移动摄像头
                if b > cy:
                    print("Move the camera down.")
                # 否则，说明直线在画面下半部分，应该向上移动摄像头
                else:
                    print("Move the camera up.")
                # 根据直线的中点和画面中心点的横坐标差，判断应该往左还是往右移动摄像头
                mx = (x1 + x2) // 2
                if mx > cx:
                    print("Move the camera right.")
                elif mx < cx:
                    print("Move the camera left.")
                else:
                    print("No need to move the camera horizontally.")
            # 计算直线与水平方向的夹角
            a = angle(k)
            # 输出夹角信息
            print(f"The angle between the line and the horizontal direction is {a:.2f} degrees.")
            # 输出宽度信息
            print(f"The width of the line is {w:.2f} pixels.")
            # 在画面上绘制最宽的直线，颜色为红色，粗细为2像素
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    
    # 在窗口上显示画面
    cv2.imshow("Video", frame)
    # 等待按键输入，如果按下q键，退出循环
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# 释放视频捕捉对象和窗口资源
cap.release()
cv2.destroyAllWindows()