#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
import math
import json

class IntegratedDeliverySystem:
    def __init__(self):
        rospy.init_node('integrated_delivery_system')
        
        # 發布器和訂閱器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/delivery_status', String, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.task_sub = rospy.Subscriber('/delivery_task', Int32, self.task_callback)
        
        # 機器人狀態
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # 預設位置
        self.locations = {
            'kitchen': {'x': -3.0, 'y': 0.0, 'name': '廚房'},
            'table1': {'x': 2.0, 'y': 2.0, 'name': '1號桌'},
            'table2': {'x': 2.0, 'y': -2.0, 'name': '2號桌'},
            'table3': {'x': -1.0, 'y': 0.0, 'name': '3號桌'},
            'home': {'x': 0.0, 'y': 0.0, 'name': '待命區'}
        }
        
        # 任務管理
        self.task_queue = []
        self.current_task = None
        self.state = 'IDLE'  # IDLE, MOVING, PICKUP, DELIVERY
        
        # 統計資料
        self.completed_tasks = 0
        self.total_distance = 0.0
        self.start_time = rospy.Time.now()
        
    def odom_callback(self, msg):
        """更新機器人位置"""
        old_x, old_y = self.current_x, self.current_y
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # 計算移動距離
        distance = math.sqrt((self.current_x - old_x)**2 + (self.current_y - old_y)**2)
        self.total_distance += distance
        
        # 獲取方向
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_theta = yaw
    
    def task_callback(self, msg):
        """接收新任務"""
        table_number = msg.data
        self.task_queue.append(table_number)
        self.publish_status(f"新任務：送餐到{table_number}號桌")
        rospy.loginfo(f"收到任務：送餐到桌號 {table_number}")
    
    def publish_status(self, message):
        """發布狀態更新"""
        status = {
            'message': message,
            'state': self.state,
            'completed_tasks': self.completed_tasks,
            'queue_length': len(self.task_queue),
            'total_distance': round(self.total_distance, 2),
            'runtime': (rospy.Time.now() - self.start_time).to_sec()
        }
        self.status_pub.publish(json.dumps(status))
    
    def move_to_position(self, target_x, target_y, location_name="", tolerance=0.3):
        """移動到指定位置"""
        self.state = 'MOVING'
        self.publish_status(f"移動到{location_name}")
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            
            if distance < tolerance:
                self.stop()
                rospy.loginfo(f"到達{location_name}")
                return True
            
            # 計算角度
            target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
            angle_diff = target_angle - self.current_theta
            
            # 正規化角度
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            cmd = Twist()
            
            if abs(angle_diff) > 0.2:
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                cmd.linear.x = min(0.3, distance * 0.5)
                cmd.angular.z = angle_diff * 0.5
            
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        return False
    
    def stop(self):
        """停止機器人"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def execute_delivery(self, table_number):
        """執行完整的送餐流程"""
        rospy.loginfo(f"開始執行送餐任務到{table_number}號桌")
        
        # 1. 前往廚房
        kitchen = self.locations['kitchen']
        if not self.move_to_position(kitchen['x'], kitchen['y'], kitchen['name']):
            return False
        
        # 2. 取餐
        self.state = 'PICKUP'
        self.publish_status("正在取餐")
        rospy.sleep(3)
        
        # 3. 送餐到桌子
        table_key = f'table{table_number}'
        if table_key not in self.locations:
            rospy.logwarn(f"未知的桌號: {table_number}")
            return False
        
        table = self.locations[table_key]
        if not self.move_to_position(table['x'], table['y'], table['name']):
            return False
        
        # 4. 放置餐點
        self.state = 'DELIVERY'
        self.publish_status(f"正在{table['name']}放置餐點")
        rospy.sleep(2)
        
        # 5. 返回待命區
        home = self.locations['home']
        self.move_to_position(home['x'], home['y'], home['name'])
        
        # 更新統計
        self.completed_tasks += 1
        self.state = 'IDLE'
        self.publish_status(f"完成送餐任務（總計：{self.completed_tasks}）")
        
        return True
    
    def run(self):
        """主循環"""
        rate = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            # 處理任務隊列
            if self.task_queue and self.state == 'IDLE':
                self.current_task = self.task_queue.pop(0)
                self.execute_delivery(self.current_task)
                self.current_task = None
            
            rate.sleep()

def main():
    system = IntegratedDeliverySystem()
    rospy.loginfo("整合送餐系統已啟動！")
    
    try:
        system.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
