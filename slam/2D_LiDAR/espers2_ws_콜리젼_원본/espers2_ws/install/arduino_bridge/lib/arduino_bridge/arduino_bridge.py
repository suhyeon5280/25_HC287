#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import time
import math
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    """
    오일러 각 (roll, pitch, yaw)을 쿼터니언으로 변환하는 함수
    """
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/arduino') #/dev/arduino
        self.declare_parameter('baudrate', 115200)

        self.wheel_base = 0.36
        self.wheel_radius = 0.054
        self.ticks_per_rev = 1632

        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.get_logger().info(f"Connecting to Arduino on {port} at {baud} baud...")
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            self.ser.flushInput()
            self.get_logger().info("Successfully connected to Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_odometry)

    def cmd_vel_callback(self, msg):
        v_r = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
        v_l = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        cmd = f"s,{v_l:.2f},{v_r:.2f}\n"
        self.ser.write(cmd.encode())

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        line = "" # line 변수 초기화

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()

                if not line:
                    return

                # === 안정성 강화 로직 시작 ===
                if line.startswith('e,') and line.endswith(','):
                    parts = line.split(',')
                    if len(parts) == 4: # ['e', 'ticks_l', 'ticks_r', '']
                        current_left_ticks = int(parts[1])
                        current_right_ticks = int(parts[2])

                        # --- 여기서부터는 기존 오도메트리 계산 로직 ---
                        if self.last_left_ticks == 0 and self.last_right_ticks == 0:
                            self.last_left_ticks = current_left_ticks
                            self.last_right_ticks = current_right_ticks
                            return

                        delta_left_ticks = current_left_ticks - self.last_left_ticks
                        delta_right_ticks = current_right_ticks - self.last_right_ticks

                        dist_left = (delta_left_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
                        dist_right = (delta_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

                        self.last_left_ticks = current_left_ticks
                        self.last_right_ticks = current_right_ticks

                        delta_dist = (dist_right + dist_left) / 2.0
                        delta_th = (dist_right - dist_left) / self.wheel_base

                        self.x += delta_dist * math.cos(self.th + delta_th / 2.0)
                        self.y += delta_dist * math.sin(self.th + delta_th / 2.0)
                        self.th += delta_th
                        
                        vx = delta_dist / dt if dt > 0 else 0.0
                        vth = delta_th / dt if dt > 0 else 0.0

                        # TF 발행
                        t = TransformStamped()
                        t.header.stamp = current_time.to_msg()
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_link'
                        t.transform.translation.x = self.x
                        t.transform.translation.y = self.y
                        q = quaternion_from_euler(0, 0, self.th)
                        t.transform.rotation.x = q[0]
                        t.transform.rotation.y = q[1]
                        t.transform.rotation.z = q[2]
                        t.transform.rotation.w = q[3]
                        self.tf_broadcaster.sendTransform(t)

                        # Odometry 토픽 발행
                        odom = Odometry()
                        odom.header.stamp = current_time.to_msg()
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_link'
                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.orientation.x = q[0]
                        odom.pose.pose.orientation.y = q[1]
                        odom.pose.pose.orientation.z = q[2]
                        odom.pose.pose.orientation.w = q[3]
                        odom.twist.twist.linear.x = vx
                        odom.twist.twist.angular.z = vth
                        self.odom_pub.publish(odom)
                    else:
                        self.get_logger().warn(f"Ignoring malformed packet (wrong part count): {line}")
                else:
                    self.get_logger().warn(f"Ignoring malformed packet (bad format): {line}")
                    self.ser.flushInput() # 버퍼를 비워 동기화 시도
                # === 안정성 강화 로직 끝 ===

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Parsing error ({e}), discarding line: '{line}'")
            self.ser.flushInput() # 버퍼를 비워 동기화 시도
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in publish_odometry: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial port closed.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()










# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import serial
# from geometry_msgs.msg import Twist, TransformStamped
# from nav_msgs.msg import Odometry
# import time
# import math
# from tf2_ros import TransformBroadcaster

# class ArduinoBridge(Node):
#     def __init__(self):
#         super().__init__('arduino_bridge')

#         # --- 파라미터 선언 ---
#         self.declare_parameter('port', '/dev/arduino') # udev 규칙으로 설정한 포트 사용
#         self.declare_parameter('baudrate', 115200)

#         # --- 로봇의 물리적 특성 (실제 로봇에 맞게 수정 필수) ---
#         self.wheel_base = 0.28      # 바퀴 간 거리 (m)
#         self.wheel_radius = 0.054   # 바퀴 반지름 (m)
#         self.ticks_per_rev = 1632   # 바퀴 1회전 당 엔코더 틱 수

#         # --- 변수 초기화 ---
#         self.last_left_ticks = 0
#         self.last_right_ticks = 0
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.last_time = self.get_clock().now()

#         # --- 시리얼 포트 설정 ---
#         port = self.get_parameter('port').value
#         baud = self.get_parameter('baudrate').value
#         self.get_logger().info(f"Connecting to Arduino on {port} at {baud} baud...")
#         try:
#             self.ser = serial.Serial(port, baud, timeout=0.1)
#             time.sleep(2)
#             self.get_logger().info("Successfully connected to Arduino.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to connect to Arduino: {e}")
#             rclpy.shutdown()
#             return

#         # --- ROS 인터페이스 설정 ---
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # --- 메인 루프: 주기적으로 시리얼 읽기 및 오도메트리 발행 ---
#         self.timer = self.create_timer(0.02, self.publish_odometry) # 50Hz

#     def cmd_vel_callback(self, msg):
#         # 받은 속도 명령을 바퀴 속도로 변환하여 아두이노에 전송
#         v_r = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
#         v_l = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        
#         # 아두이노와 약속된 형식으로 명령 전송 (예: "s,v_l,v_r\n")
#         cmd = f"s,{v_l:.2f},{v_r:.2f}\n"
#         self.ser.write(cmd.encode())

#     def publish_odometry(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time

#         try:
#             # 아두이노로부터 엔코더 데이터 읽기 (예: "e,123,456,\n")
#             line = self.ser.readline().decode('utf-8').strip()
#             if line.startswith('e,'):
#                 parts = line.split(',')
#                 current_left_ticks = int(parts[1])
#                 current_right_ticks = int(parts[2])

#                 # 처음 실행 시 초기값 설정
#                 if self.last_left_ticks == 0 and self.last_right_ticks == 0:
#                     self.last_left_ticks = current_left_ticks
#                     self.last_right_ticks = current_right_ticks
#                     return

#                 # 엔코더 틱 변화량 계산
#                 delta_left_ticks = current_left_ticks - self.last_left_ticks
#                 delta_right_ticks = current_right_ticks - self.last_right_ticks

#                 # 이동 거리 계산
#                 dist_left = (delta_left_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
#                 dist_right = (delta_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

#                 self.last_left_ticks = current_left_ticks
#                 self.last_right_ticks = current_right_ticks

#                 # 로봇의 이동량 및 방향 변화량 계산
#                 delta_dist = (dist_right + dist_left) / 2.0
#                 delta_th = (dist_right - dist_left) / self.wheel_base

#                 # 로봇의 위치와 방향 업데이트
#                 self.x += delta_dist * math.cos(self.th + delta_th / 2.0)
#                 self.y += delta_dist * math.sin(self.th + delta_th / 2.0)
#                 self.th += delta_th
                
#                 # 속도 계산
#                 vx = delta_dist / dt if dt > 0 else 0.0
#                 vth = delta_th / dt if dt > 0 else 0.0

#                 # --- TF 발행 (odom -> base_link) ---
#                 t = TransformStamped()
#                 t.header.stamp = current_time.to_msg()
#                 t.header.frame_id = 'odom'
#                 t.child_frame_id = 'base_link'
#                 t.transform.translation.x = self.x
#                 t.transform.translation.y = self.y
#                 q = quaternion_from_euler(0, 0, self.th)
#                 t.transform.rotation.x = q[0]
#                 t.transform.rotation.y = q[1]
#                 t.transform.rotation.z = q[2]
#                 t.transform.rotation.w = q[3]
#                 self.tf_broadcaster.sendTransform(t)

#                 # --- /odom 토픽 발행 ---
#                 odom = Odometry()
#                 odom.header.stamp = current_time.to_msg()
#                 odom.header.frame_id = 'odom'
#                 odom.child_frame_id = 'base_link'
#                 odom.pose.pose.position.x = self.x
#                 odom.pose.pose.position.y = self.y
#                 odom.pose.pose.orientation.x = q[0]
#                 odom.pose.pose.orientation.y = q[1]
#                 odom.pose.pose.orientation.z = q[2]
#                 odom.pose.pose.orientation.w = q[3]
#                 odom.twist.twist.linear.x = vx
#                 odom.twist.twist.angular.z = vth
#                 self.odom_pub.publish(odom)

#         except (ValueError, IndexError, UnicodeDecodeError):
#             # 시리얼 데이터 파싱 중 오류가 발생하면 무시하고 다음으로 넘어감
#             pass
#         except Exception as e:
#             self.get_logger().error(f"Error in publish_odometry: {e}")

# def quaternion_from_euler(roll, pitch, yaw):
#     cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
#     q = [0] * 4
#     q[0] = sr * cp * cy - cr * sp * sy
#     q[1] = cr * sp * cy + sr * cp * sy
#     q[2] = cr * cp * sy - sr * sp * cy
#     q[3] = cr * cp * cy + sr * sp * sy
#     return q

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArduinoBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


















# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import serial
# import time
# import math

# from geometry_msgs.msg import Twist, TransformStamped
# from nav_msgs.msg import Odometry
# from tf2_ros import TransformBroadcaster

# class ArduinoBridge(Node):
#     def __init__(self):
#         super().__init__('arduino_bridge')

#         # --- 파라미터 선언 ---
#         self.declare_parameter('port', '/dev/ttyACM0')
#         self.declare_parameter('baudrate', 115200)

#         # --- 로봇의 물리적 특성 (바퀴 속도 계산에만 사용) ---
#         self.WHEEL_BASE = 0.28  # 바퀴 사이의 거리 (미터)

#         # --- 시리얼 포트 설정 ---
#         port = self.get_parameter('port').value
#         baud = self.get_parameter('baudrate').value
#         self.get_logger().info(f"Connecting to Arduino on {port} at {baud} baud...")
#         try:
#             self.ser = serial.Serial(port, baud, timeout=0.1)
#             time.sleep(2)
#             self.get_logger().info("Successfully connected to Arduino.")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to connect to Arduino: {e}")
#             # 아두이노 없이도 RViz 시뮬레이션은 가능하도록 주석 처리
#             # rclpy.shutdown()
#             # return
#             self.ser = None


#         # --- ROS 인터페이스 설정 ---
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # --- 가상 Odometry 계산을 위한 변수 ---
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.current_twist = Twist() # 현재 속도 명령을 저장할 변수
#         self.last_time = self.get_clock().now()

#         # --- 메인 루프: 20Hz (0.05초마다) Odometry 발행 ---
#         self.create_timer(0.05, self.publish_odom_callback)

#     def cmd_vel_callback(self, msg):
#         # 1. 받은 속도 명령을 저장
#         self.current_twist = msg

#         # 2. 실제 아두이노로 명령 전송 (모터 구동용)
#         if self.ser and self.ser.is_open:
#             v = msg.linear.x
#             w = msg.angular.z
#             v_r = v + (w * self.WHEEL_BASE / 2.0)
#             v_l = v - (w * self.WHEEL_BASE / 2.0)
#             cmd = f"{v_r:.2f},{v_l:.2f}\n"
#             self.ser.write(cmd.encode())

#     def publish_odom_callback(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9

#         # 저장된 속도 명령으로 이동량 계산
#         vx = self.current_twist.linear.x
#         vth = self.current_twist.angular.z

#         delta_dist = vx * dt
#         delta_theta = vth * dt
        
#         # 로봇의 위치 및 자세 업데이트
#         self.x += delta_dist * math.cos(self.theta)
#         self.y += delta_dist * math.sin(self.theta)
#         self.theta += delta_theta
#         self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) # 정규화

#         # --- TF 발행 ---
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         q = quaternion_from_euler(0, 0, self.theta)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         self.tf_broadcaster.sendTransform(t)

#         # --- /odom 토픽 발행 ---
#         odom_msg = Odometry()
#         odom_msg.header.stamp = current_time.to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.orientation.w = q[3]
#         odom_msg.pose.pose.orientation.x = q[0]
#         odom_msg.pose.pose.orientation.y = q[1]
#         odom_msg.pose.pose.orientation.z = q[2]
#         odom_msg.twist.twist.linear.x = vx
#         odom_msg.twist.twist.angular.z = vth
#         self.odom_pub.publish(odom_msg)

#         self.last_time = current_time

# # 오일러 각을 쿼터니언으로 변환하는 함수
# def quaternion_from_euler(roll, pitch, yaw):
#     cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5);
#     cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5);
#     cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5);
#     q = [0] * 4
#     q[0] = sr * cp * cy - cr * sp * sy; q[1] = cr * sp * cy + sr * cp * sy;
#     q[2] = cr * cp * sy - sr * sp * cy; q[3] = cr * cp * cy + sr * sp * sy;
#     return q

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArduinoBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()














# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import serial
# from geometry_msgs.msg import Twist
# import time

# class ArduinoBridge(Node):
#     def __init__(self):
#         super().__init__('arduino_bridge')

#         # 시리얼 포트 설정
#         self.declare_parameter('port', '/dev/arduino')
#         self.declare_parameter('baudrate', 115200)

#         port = self.get_parameter('port').value
#         baud = self.get_parameter('baudrate').value

#         self.get_logger().info(f"Connecting to Arduino on {port} at {baud} baud...")
#         self.ser = serial.Serial(port, baud, timeout=0.1)
#         time.sleep(2)  # 아두이노 리셋 대기

#         # /cmd_vel 구독
#         self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

#     def cmd_vel_callback(self, msg):
#         # 간단한 차동 구동 변환 (m/s 단위 그대로 전송)
#         # 선속도: msg.linear.x
#         # 각속도: msg.angular.z
#         wheel_base = 0.3  # 바퀴 간 거리 (m)
#         v_r = msg.linear.x + (msg.angular.z * wheel_base / 2.0)
#         v_l = -(msg.linear.x - (msg.angular.z * wheel_base / 2.0))

#         # 아두이노로 전송 (형식: "v_r,v_l,\n")
#         cmd = f"{v_r:.2f},{v_l:.2f},\n"
#         self.ser.write(cmd.encode())

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArduinoBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


