import cv2


def Coordinate2023(xmeter: float, ymeter: float) -> float:
    width, height = 1280, 720
    x0 = xmeter * 10.0 * 18.0 + 63.0
    y0 = ymeter * 10.0 * 18.0 + 63.0
    a = x0
    b = y0
    x1 = -b
    y1 = a
    x = x1
    y = height - y1
    return int(x), int(y)

def orbit2023(img):
    # img = cv2.imread("./2023/864x720.jpg")
    # 循环读取实时变化的坐标点
    while True:
        cv2.imshow("image", img)
        # 从外部接收实时变化的坐标点(ROS topic)
        # x, y = receive_coordinate()
        # 从命令行输入模拟外部输入的试试变化的坐标
        x, y = input("请输入坐标点(x,y):").split(",")
        x, y = float(x), float(y)
        # 将坐标点转换为图像坐标系中的坐标
        img_x, img_y = Coordinate2023(x, y)
        # 在图像上绘制坐标点
        imgorbit = cv2.circle(img, (img_x, img_y), radius=7, color=(0, 0, 255), thickness=-1)
        # 显示图像
        cv2.imshow("image", imgorbit)
    return imgorbit

# 关闭窗口
cv2.destroyAllWindows()
