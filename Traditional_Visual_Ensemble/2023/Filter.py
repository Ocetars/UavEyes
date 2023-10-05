import rospy
from geometry_msgs.msg import Point
import math


def filter_point(new_point):
    global last_point, error_count
    if last_point is None:
        last_point = new_point
        return new_point
    else:
        # 计算偏差值
        deviation = math.sqrt((new_point[0] - last_point[0]) ** 2 + (new_point[1] - last_point[1]) ** 2)
        if deviation < threshold:
            last_point = new_point
            error_count = 0
            return new_point
        else:
            error_count += 1
            if error_count >= max_error_count:
                last_point = new_point
                error_count = 0
                return new_point
            else:
                return None

# 回调函数，处理接收到的 Point 消息
def point_callback(point_msg):
    # 提取 x, y值
    X_value = point_msg.x
    Y_value = point_msg.y
    new_point = (X_value, Y_value)
    output_point = filter_point(new_point)
    if output_point is not None:
        point_ros = Point()
        point_ros.x = output_point[0]
        point_ros.y = output_point[1]
        rospy.loginfo(point_ros)
        pub.publish(point_ros)

if __name__ == '__main__':
    rospy.init_node('filter_node')
    pub = rospy.Publisher('filter_out', Point, queue_size=10)
    threshold = 150  # 阈值
    last_point = None  # 上一个坐标点
    error_count = 0  # 连续的误差次数
    max_error_count = 60  # 最大的连续误差次数
    sub = rospy.Subscriber('fire_point', Point, point_callback)
    rospy.spin()