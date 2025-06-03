#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

def status_callback(msg):
    """顯示系統狀態"""
    try:
        status = json.loads(msg.data)
        print("\n" + "="*50)
        print(f"🤖 送餐機器人狀態")
        print("="*50)
        print(f"📍 當前狀態: {status['state']}")
        print(f"💬 訊息: {status['message']}")
        print(f"✅ 完成任務: {status['completed_tasks']}")
        print(f"📋 等待任務: {status['queue_length']}")
        print(f"📏 總移動距離: {status['total_distance']} 公尺")
        print(f"⏱️  運行時間: {int(status['runtime'])} 秒")
        print("="*50)
    except:
        print(f"狀態: {msg.data}")

def main():
    rospy.init_node('status_monitor')
    rospy.Subscriber('/delivery_status', String, status_callback)
    rospy.loginfo("狀態監控器已啟動")
    rospy.spin()

if __name__ == '__main__':
    main()
