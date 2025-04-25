# gesture_ws
# 注意！！！！realsense包不齊全 要額外下載
這是可以利用深度攝影機D435i透過Mediapipe進行手勢辨識並控制機器人移動
# 啟動機器人
 rosrun tracer_bring setup_can2usb.bash
# 啟動深度攝影機
 roslaunch realsense2_camera rs_camera.launch 
# 手勢辨識
 rosrun gesture_recognition gesture_recognition.py
# 辨識後控制機器人
 rosrun gesture_recognition gesture_controll
