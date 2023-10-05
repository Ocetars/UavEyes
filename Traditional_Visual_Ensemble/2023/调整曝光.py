import cv2

# 打开摄像头
cap = cv2.VideoCapture(2)

while True:
    # 读取摄像头画面
    ret, frame = cap.read()

    # 自动白平衡
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    # 显示画面
    cv2.imshow('frame', final)

    # 按下q键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头
cap.release()

# 关闭窗口
cv2.destroyAllWindows()