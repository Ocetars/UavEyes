import math
import cv2


# 创建一个视频捕捉对象，参数为摄像头编号，0为默认摄像头
cap = cv2.VideoCapture(0)

# 设置摄像头分辨率和视角
camera_width = 1920 # 摄像头宽度（像素）
camera_height = 1080 # 摄像头高度（像素）
camera_angle = 60 # 摄像头视角（度）

# 计算摄像头中心点坐标
camera_center_x = camera_width / 2
camera_center_y = camera_height / 2

# 循环获取摄像头图像
while True:
    # 获取摄像头图像
    image = get_camera_image()

    # 假设已知呼啦圈的中心坐标为(a,b)
    hoop_center_x = a
    hoop_center_y = b

    # 计算摄像头距离呼啦圈的距离（假设呼啦圈直径为1米）
    hoop_diameter = 1000 # 呼啦圈直径（毫米）
    hoop_width = math.sqrt((hoop_center_x - camera_center_x) ** 2 + (hoop_center_y - camera_center_y) ** 2) # 呼啦圈在图像中的宽度（像素）
    camera_distance = (hoop_diameter * camera_width) / (2 * hoop_width * math.tan(math.radians(camera_angle / 2))) # 摄像头距离呼啦圈的距离（毫米）

    # 比较摄像头中心点和呼啦圈中心点的坐标差，输出移动指令
    x_diff = hoop_center_x - camera_center_x # x方向的坐标差（像素）
    y_diff = hoop_center_y - camera_center_y # y方向的坐标差（像素）
    x_threshold = camera_width * 0.05 # x方向的容忍误差（像素）
    y_threshold = camera_height * 0.05 # y方向的容忍误差（像素）

    if abs(x_diff) > x_threshold: # 如果x方向偏离中心，需要左右移动
        direction = 'left' if x_diff > 0 else 'right' # 判断左右方向
        print(f'Move {direction}')
    else: # 如果x方向居中，不需要移动
        print('Stay in the middle')

    if abs(y_diff) > y_threshold: # 如果y方向偏离中心，需要上下移动
        direction = 'down' if y_diff > 0 else 'up' # 判断上下方向
        print(f'Move {direction}')
    else: # 如果y方向居中，不需要移动
        print('Stay in the middle')

    # 根据摄像头距离呼啦圈的距离，输出速度指令
    distance_threshold = 1000 # 距离阈值（毫米）
    
    if abs(camera_distance - distance_threshold) > distance_threshold * 0.1: # 如果距离偏离阈值，需要加速或减速
        speed = 'up' if camera_distance > distance_threshold else 'down' # 判断加速或减速
        print(f'Speed {speed}')