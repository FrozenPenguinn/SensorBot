import vrep
import time
import numpy as np
import cv2

# ---------------连接到服务器----------------
# 以防万一，停止所有先前的连接
vrep.simxFinish(-1)
# 连接到vrep服务器
vrepClientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
# 检查客户端连接服务器是否成功
if vrepClientID != -1:
    print("Connected successful")
else:
    raise Exception("Failed to connect")

# -----------------------------------------
# 开始仿真
vrep.simxStartSimulation(vrepClientID, vrep.simx_opmode_oneshot)

# ----------------初始化---------------------
# 获取关节的句柄
res, fl_handle = vrep.simxGetObjectHandle(vrepClientID, 'fl_motor', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of front left wheel")
res, fr_handle = vrep.simxGetObjectHandle(vrepClientID, 'fr_motor', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of front right wheel")
res, cam_handle = vrep.simxGetObjectHandle(vrepClientID, 'vision_sensor', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of vision sensor")
res, ls_left_handle = vrep.simxGetObjectHandle(vrepClientID, 'line_sensor_left', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of linetrace_sensor_left")
res, ls_mid_handle = vrep.simxGetObjectHandle(vrepClientID, 'line_sensor_middle', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of linetrace_sensor_mid")
res, ls_right_handle = vrep.simxGetObjectHandle(vrepClientID, 'line_sensor_right', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of linetrace_sensor_right")
res, ultrasound_handle = vrep.simxGetObjectHandle(vrepClientID, 'ultrasound_sensor', vrep.simx_opmode_blocking)
if res != vrep.simx_return_ok:
    raise Exception("Cannot get handle of ultrasound sensor")

# 初始化变量
fl_velocity = 0
fr_velocity = 0

# ---------------------执行器脚本--------------
# 控制关节运动
def set_wheels(fl_velocity, fr_velocity):
    vrep.simxSetJointTargetVelocity(vrepClientID, fl_handle, fl_velocity, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(vrepClientID, fr_handle, fr_velocity, vrep.simx_opmode_oneshot)

set_wheels(fl_velocity, fr_velocity)

# 加速度计
ret, accel = vrep.simxGetStringSignal(vrepClientID, 'accel', vrep.simx_opmode_streaming)
time.sleep(0.1)
while True:
    ret, accel = vrep.simxGetStringSignal(vrepClientID, 'accel', vrep.simx_opmode_buffer)
    if ret == vrep.simx_return_ok:
        accelData = vrep.simxUnpackFloats(accel)
        print(accelData)

'''
# 摄像头输出
#cv2.namedWindow('cam_view', cv2.WINDOW_NORMAL) # 创建显示窗口
#cv2.resizeWindow('cam_view', 256, 256)
# 第一次的数据是无效数据
ret, resolution, image = vrep.simxGetVisionSensorImage(vrepClientID, cam_handle, 0, vrep.simx_opmode_streaming)
time.sleep(0.1)
# 获取有效数据
while True:
    ret, resolution, image = vrep.simxGetVisionSensorImage(vrepClientID, cam_handle, 0, vrep.simx_opmode_buffer)
    sensorImage = np.array(image,dtype = np.uint8)
    sensorImage.resize([resolution[0],resolution[1],3])#整理矩阵
    sensorImage = cv2.flip(cv2.cvtColor(sensorImage,cv2.COLOR_RGB2BGR),0)
    #cv2.imshow('cam_view',sensorImage)#显示图像
    # 控制
    gray = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
    ret, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    color = dst[250]
    black_count = np.sum(color == 0)
    black_index = np.where(color == 0)
    if (black_count == 0):
        black_count = 1
    center = (black_index[0][black_count-1] + black_index[0][0]) / 2
    direction = center - 128
    set_wheels(2+0.05*direction, 2-0.05*direction)
'''
'''
# 超声波传感器避障
# 第一次的数据是无效数据
ret, state, dist_1, detected_obj_handle, arr2 = vrep.simxReadProximitySensor(vrepClientID, ultrasound_handle, vrep.simx_opmode_streaming)
time.sleep(0.1)
# 获取有效数据
while True:
    ret, state, dist_1, detected_obj_handle, arr2 = vrep.simxReadProximitySensor(vrepClientID, ultrasound_handle, vrep.simx_opmode_buffer)
    if (dist_1[1] > 1e-3):
        set_wheels(-4, -2)
        time.sleep(0.3)
    else:
        set_wheels(2, 2)
'''
'''
# 灰度传感器巡线
line_sensor_handles = [ls_left_handle, ls_mid_handle, ls_right_handle]
reading = [0,0,0] # 初始化灰度传感器读数
# 第一次的数据是无效数据
for i in range(0,3):
    ret, status, data = vrep.simxReadVisionSensor(vrepClientID, line_sensor_handles[i], vrep.simx_opmode_streaming)
time.sleep(0.1)
# 获取有效数据
while True:
    for i in range(0,3):
        ret, status, data = vrep.simxReadVisionSensor(vrepClientID, line_sensor_handles[i], vrep.simx_opmode_buffer)
        reading[i] = data[0][11]
    if (reading[0] < 0.2):
        set_wheels(-10, 4)
    elif (reading[2] < 0.2):
        set_wheels(4, -10)
    elif (reading[0] > 0.5 and reading[1] > 0.5 and reading[2] > 0.5):
        set_wheels(-3, -3)
    else:
        set_wheels(3, 3)
'''
# 让仿真飞一会儿
time.sleep(5)
set_wheels(0, 0)
time.sleep(0.1)

# ---------------------------
# 停止仿真
vrep.simxStopSimulation(vrepClientID, vrep.simx_opmode_oneshot)

# ------------清理脚本------------------
# 确保停止模拟，阻塞等待
vrep.simxGetPingTime(vrepClientID)
# 断开客户端与服务器的连接
vrep.simxFinish(vrepClientID)
