import cv2

# 创建一个VideoCapture对象，从摄像头获取图像
cap = cv2.VideoCapture(0)

while True:
    # 读取摄像头画面
    ret, frame = cap.read()
    # 显示摄像头画面
    cv2.imshow("Camera", frame)
    # 按下q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break