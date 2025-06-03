#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math

class DeliveryRobot:
    def __init__(self):
        rospy.init_node('delivery_robot')
        
        # publishers & subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub    = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub   = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # 機器人當前狀態
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        
        # LaserScan 資料 (保留整張，供判斷是否真正擋到)
        self.last_ranges = []
        self.angle_min   = 0.0
        self.angle_inc   = 0.0
        
        # 正前方避障判斷 (可選，如果只靠卡住判斷就不需要)
        self.obstacle_detected     = False
        self.min_obstacle_distance = float('inf')
        
        # 預設各地點座標
        self.locations = {
            'kitchen': {'x': -4.0, 'y':  0.0},
            'table1' : {'x':  0.8, 'y':  1.2},
            'table2' : {'x':  0.8, 'y': -1.2},
            'table3' : {'x': -2.0, 'y':  0.0},
            'home'   : {'x':  0.0, 'y':  0.0},
        }
        
        # 任務佇列
        self.current_task = None
        self.task_queue   = []
    
    def odom_callback(self, msg):
        """更新位置與朝向 (yaw)"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_theta = yaw
    
    def laser_callback(self, msg):
        """
        儲存整張 LaserScan，若想用正前方簡易避障，也可像下面這樣擷取 ±10° 判定：
        """
        self.last_ranges = msg.ranges
        self.angle_min   = msg.angle_min
        self.angle_inc   = msg.angle_increment
        
        # ↓ 可保留簡易正前方判障，或刪掉都可以
        total = len(msg.ranges)
        center_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        span = int(math.radians(10) / msg.angle_increment)
        front = msg.ranges[max(0, center_idx - span): min(total, center_idx + span)]
        valid = [r for r in front if 0.2 < r < 10.0]
        if valid:
            self.min_obstacle_distance = min(valid)
            self.obstacle_detected = (self.min_obstacle_distance < 0.2)
        else:
            self.min_obstacle_distance = float('inf')
            self.obstacle_detected = False
    
    def get_distance(self, tx, ty):
        """計算 (tx,ty) 與當前位置的距離"""
        return math.hypot(tx - self.current_x, ty - self.current_y)
    
    def get_angle_to_target(self, tx, ty):
        """計算當前 heading 到 (tx,ty) 的角度差 (rad)"""
        return math.atan2(ty - self.current_y, tx - self.current_x)
    
    def move_to_position(self, tx, ty, tolerance=0.5):
        """
        移動到 (tx, ty)。若卡住太久就呼叫 turn_left_90_and_forward()
        """
        rate = rospy.Rate(10)
        stuck_counter = 0
        last_pos = (self.current_x, self.current_y)
        
        while not rospy.is_shutdown():
            dist = self.get_distance(tx, ty)
            if dist < tolerance:
                self.stop()
                rospy.loginfo(f"到達: ({tx:.2f}, {ty:.2f})")
                return True
            
            # 計算偏差角度
            target_angle = self.get_angle_to_target(tx, ty)
            angle_diff   = target_angle - self.current_theta
            # 正規化到 [-π, π]
            while angle_diff > math.pi:
                angle_diff -= 2*math.pi
            while angle_diff < -math.pi:
                angle_diff += 2*math.pi
            
            # 如果想保留「正前方 <0.2m 就避障」，可取消下面註解
            # if self.obstacle_detected:
            #     rospy.logwarn("[避障] 正前方偵測到障礙，呼叫強行脫困")
            #     self.turn_left_90_and_forward()
            #     stuck_counter = 0
            #     last_pos = (self.current_x, self.current_y)
            #     continue
            
            cmd = Twist()
            # 需要大幅轉向 (>0.3 rad) 時，原地轉，不算卡住
            if abs(angle_diff) > 0.3:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                stuck_counter = 0
                last_pos = (self.current_x, self.current_y)
            else:
                # 角度已經 ok，開始前進 + 微調
                if dist > 2.0:
                    cmd.linear.x = 0.3
                elif dist > 1.0:
                    cmd.linear.x = 0.2
                else:
                    cmd.linear.x = 0.1
                cmd.angular.z = angle_diff * 0.5
                
                # 只有在真正嘗試前進 (cmd.linear.x>0) 時才判卡住
                current_pos = (self.current_x, self.current_y)
                moved = math.hypot(current_pos[0] - last_pos[0],
                                   current_pos[1] - last_pos[1])
                if moved < 0.05 and cmd.linear.x > 0:
                    stuck_counter += 1
                else:
                    stuck_counter = 0
                    last_pos = current_pos
                
                # 如果持續約 2 秒 (20 個迴圈) 都沒移動，就判定卡住
                if stuck_counter > 20:
                    rospy.logwarn("[長時間卡住] 執行左轉 90° + 前進脫困")
                    self.turn_left_90_and_forward()
                    stuck_counter = 0
                    last_pos = (self.current_x, self.current_y)
                    continue
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # ROS shutdown 時停車
        self.stop()
        return False
    
    def turn_left_90_and_forward(self):
        """
        (1) 左轉 90°
        (2) 向前衝 1 公尺
        """
        # Step 1: 左轉 90°
        turn_cmd = Twist()
        turn_cmd.angular.z = 0.5  # 0.5 rad/s
        self.cmd_vel_pub.publish(turn_cmd)
        # time = 90° / 0.5rad/s = (pi/2) / 0.5 = pi 秒 ≈ 1.57s
        rospy.sleep(math.pi / 0.5)
        self.stop()
        rospy.sleep(0.1)
        
        # Step 2: 前進一大段 (1m)
        fwd_cmd = Twist()
        fwd_cmd.linear.x = 0.3   # 0.3 m/s
        self.cmd_vel_pub.publish(fwd_cmd)
        # time = 1.0m / 0.3m/s ≈ 3.33s
        rospy.sleep(1.0 / 0.3)
        self.stop()
        rospy.sleep(0.1)
    
    def stop(self):
        """立刻停車"""
        self.cmd_vel_pub.publish(Twist())
    
    def go_to_location(self, name):
        if name not in self.locations:
            rospy.logwarn(f"未知地點：{name}")
            return False
        coord = self.locations[name]
        rospy.loginfo(f"前往 {name}：({coord['x']:.2f}, {coord['y']:.2f})")
        return self.move_to_position(coord['x'], coord['y'])
    
    def deliver_food(self, table_number):
        """送餐流程：廚房 → 桌子 → 待命"""
        rospy.loginfo(f"開始送餐到桌 {table_number}")
        # 1. 前往廚房
        if not self.go_to_location('kitchen'):
            rospy.logerr("無法到廚房")
            return False
        rospy.loginfo("取餐中...")
        rospy.sleep(3)
        
        # 2. 送到指定桌
        tbl = f"table{table_number}"
        if not self.go_to_location(tbl):
            rospy.logerr(f"無法到 {tbl}")
            return False
        rospy.loginfo("放置餐點...")
        rospy.sleep(2)
        
        # 3. 回 home
        self.go_to_location('home')
        rospy.loginfo("送餐完成")
        return True
    
    def add_task(self, table_number):
        self.task_queue.append(table_number)
        rospy.loginfo(f"新增任務：送餐到桌 {table_number}")
    
    def process_tasks(self):
        if self.task_queue and not self.current_task:
            self.current_task = self.task_queue.pop(0)
            success = self.deliver_food(self.current_task)
            self.current_task = None
            return success
        return True

def main():
    robot = DeliveryRobot()
    rospy.loginfo("送餐機器人啟動")
    
    rospy.sleep(2)  # 等 odom/laser 初始化
    
    # 測試任務
    robot.add_task(1)
    robot.add_task(2)
    robot.add_task(3)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        robot.process_tasks()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
