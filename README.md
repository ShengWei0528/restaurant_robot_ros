# 送餐服務機器人 ROS 專案
[Demo Video](https://youtu.be/TXWxIsT_JnY)
## 專案描述
本專案實現了一個基於 ROS 的送餐服務機器人系統，能夠在餐廳環境中自主導航並完成送餐任務。

## 系統功能
- 餐廳環境模擬
- 機器人 SLAM 建圖
- 自主移動到指定位置
- 任務管理系統
- 基本避障功能

## 環境需求
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

## 安裝說明
1. 安裝依賴套件：
```bash
sudo apt-get update
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-navigation

複製專案到您的 catkin 工作空間：
bashcd ~/catkin_ws/src
git clone https://github.com/您的用戶名/restaurant_robot_ros.git
cd ..
catkin_make

## 專案描述
本專案實現了一個基於 ROS 的送餐服務機器人系統，能夠在餐廳環境中自主導航並完成送餐任務。

## 系統功能
- 餐廳環境模擬
- 機器人 SLAM 建圖
- 自主移動到指定位置
- 任務管理系統
- 基本避障功能

## 環境需求
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11

##使用方法

1. 啟動完整系統：

bashsource devel/setup.bash
roslaunch restaurant_robot final_demo.launch

2. 機器人會自動執行預設的送餐任務

專案結構
restaurant_robot/
├── launch/          # 啟動檔案
├── urdf/           # 機器人模型
├── worlds/         # Gazebo 環境
├── scripts/        # Python 控制程式
├── maps/           # SLAM 地圖
└── config/         # 配置檔案
