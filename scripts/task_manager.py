#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import sys

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')
        self.task_pub = rospy.Publisher('/delivery_task', Int32, queue_size=10)
        
    def send_task(self, table_number):
        """發送送餐任務"""
        msg = Int32()
        msg.data = table_number
        self.task_pub.publish(msg)
        rospy.loginfo(f"已發送任務：送餐到桌號 {table_number}")

def main():
    manager = TaskManager()
    rospy.loginfo("任務管理器已啟動！")
    rospy.loginfo("輸入桌號 (1-3) 來添加送餐任務，輸入 'q' 退出")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("請輸入桌號: ")
            if user_input.lower() == 'q':
                break
            
            table_num = int(user_input)
            if 1 <= table_num <= 3:
                manager.send_task(table_num)
            else:
                print("請輸入 1-3 的桌號")
                
        except ValueError:
            print("請輸入有效的數字")
        except KeyboardInterrupt:
            break
    
    rospy.loginfo("任務管理器已關閉")

if __name__ == '__main__':
    main()
