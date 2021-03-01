import time
from src.RemoteControl import RemoteControl
from concurrent.futures import ThreadPoolExecutor
import subprocess
import rospy
from sensor_msgs.msg import Imu, CameraInfo, TimeReference
import numpy as np
import pandas as pd
from io import StringIO
from src.TimeSync import TimeSync2

import matplotlib.pyplot as plt
import signal
import sys

HOST = '10.30.65.137'  # The smartphone's IP address

mcu_imu_time = []
mcu_imu_data = []
global stop_flag
stop_flag = 0

#global depth_cam_ts
depth_cam_ts = []#{'secs': None, 'nsecs' : None}

#global mcu_cam_ts
mcu_cam_ts = []#{'secs': None, 'nsecs' : None}

def mcu_imu_callback(data):
    dat = data.header.stamp.secs + data.header.stamp.nsecs  / 1e9
    mcu_imu_time.append(dat)

    dat = data.angular_velocity
    mcu_imu_data.append([dat.x, dat.y, dat.z])

def depth_cam_callback(data):
    if data.header.seq == 1:
        depth_cam_ts.append(data.header.stamp)
        print data.header.seq, data.header.frame_id

def mcu_cam_callback(data):
    if data.header.seq == 12:
        mcu_cam_ts.append(data.header.stamp)
        print data.header.seq, data.header.frame_id
        #global mcu_cam_ts
        #mcu_cam_ts['secs'] = data.header.stamp.secs
        #mcu_cam_ts['nsecs'] = data.header.stamp.nsecs

        
def main():


    '''msg = TimeReference()
    msg.header.frame_id = "mcu_depth_ts"
    msg.header.stamp.secs = 1
    msg.header.stamp.nsecs = 1
    msg.time_ref.secs = 1
    msg.time_ref.nsecs = 1

    msg2 = TimeReference()
    msg2.header.stamp = msg.header.stamp
    print msg2#.header.stamp

    exit()'''
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C. Exiting.')
        global stop_flag
        stop_flag = 1

    signal.signal(signal.SIGINT, signal_handler)

    remote = RemoteControl(HOST)

    launch_process = subprocess.Popen("roslaunch data_collection data_collection_ns.launch".split())
    time.sleep(3)

    rospy.init_node('listener', anonymous=True)

# 1. Twist-n-Sync 
    start_duration = 1
    main_duration = 4
    end_duration = 5
    
    with ThreadPoolExecutor(max_workers=1) as executor:
        print('imus gathering started')
        future = executor.submit(remote.get_imu, 1000 * (start_duration + main_duration + end_duration), False, True)
        #mcu_imu_listener()

        mcu_imu_listener = rospy.Subscriber("mcu_imu", Imu, mcu_imu_callback)

        time.sleep(start_duration)
        print('start shaking')
        time.sleep(main_duration)
        print('put back')
        time.sleep(end_duration)

        #rospy.signal_shutdown('it is enough')
        mcu_imu_listener.unregister()

        accel_data, gyro_data = future.result()
        print('stopped')

    # Get data from mcu imu
    mcu_gyro_data = np.asarray(mcu_imu_data) - np.asarray(mcu_imu_data)[:200].mean(axis=0) # Subtract bias in addition
    mcu_gyro_time = np.asarray(mcu_imu_time)
    print(gyro_data[:200]) # Show the problem of the first measurement

    # Get data from s10 imu
    sm_df = pd.read_csv(StringIO(unicode(gyro_data)), header=None, index_col=False)
    sm_gyro_data = sm_df.iloc[1:, :3].to_numpy()
    sm_gyro_time = sm_df.iloc[1:, 3].to_numpy() / 1e9
    
    # Equalize lengths
    min_length = min(sm_gyro_time.shape[0], mcu_gyro_time.shape[0])
    mcu_gyro_data, mcu_gyro_time, sm_gyro_data, sm_gyro_time = \
    mcu_gyro_data[:min_length], mcu_gyro_time[:min_length], \
    sm_gyro_data[:min_length], sm_gyro_time[:min_length]

    # Obtain offset
    time_sync2 = TimeSync2(
        mcu_gyro_data, sm_gyro_data, mcu_gyro_time, sm_gyro_time, False
    )
    time_sync2.resample(accuracy=1)
    time_sync2.obtain_delay()
    comp_delay2 = time_sync2.time_delay

    # Compute resulting offset
    sm_mcu_clock_offset = np.mean(sm_gyro_time - mcu_gyro_time) + comp_delay2 #sm_mcu_clock_offset = (sm_gyro_time[0] - mcu_gyro_time[0] + comp_delay2)

    # Show mean of omegas to visually oversee sync performance
    plt.ion()
    plt.plot(mcu_gyro_time, np.mean(mcu_gyro_data, axis=1))
    plt.plot(sm_gyro_time - sm_mcu_clock_offset, np.mean(sm_gyro_data, axis=1), '--')
    plt.show()
    plt.pause(2)
    plt.close()
# 2. Azure camera alignment
    depth_cam_listener = rospy.Subscriber("/azure/depth/camera_info", CameraInfo, depth_cam_callback)
    mcu_cam_listener = rospy.Subscriber("/mcu_cameras_ts", TimeReference, mcu_cam_callback)

    # Send start_mcu_cam_triggering command to mcu via mcu.cpp
    cam_align_process = subprocess.Popen("rosrun mcu_interface start_mcu_cam_trigger_client".split()) 

    # Some time needed to get a camrera frame and its info in mcu.cpp
    time.sleep(0.1)
    
    pub = rospy.Publisher('wewerqwerqewrqwerddsdasd', TimeReference, latch=True, queue_size=10)
    
    while len(mcu_cam_ts) != 1 or len(depth_cam_ts) != 1:
        time.sleep(0.01)
    depth_cam_listener.unregister()
    mcu_cam_listener.unregister()
    msg = TimeReference()
    msg.header.frame_id = "mcu_depth_ts"
    msg.header.stamp = mcu_cam_ts[0]
    msg.time_ref = depth_cam_ts[0]
    pub.publish(msg)

    # Start video on s10
    sm_remote_ts_ns, sm_frame_period_ns = remote.start_video()
    sm_remote_ts = sm_remote_ts_ns / 1e9;
    #sm_frame_period = sm_frame_period_ns / 1e9

    # Compute mcu desired timestamp
    mcu_desired_ts = sm_remote_ts - sm_mcu_clock_offset
    '''
    # Save some info 
    print "comp_delay2        ", comp_delay2
    print "sm_mcu_clock_offset", sm_mcu_clock_offset
    print "sm_remote_ts       ", sm_remote_ts
    #print "sm_frame_period    ", sm_frame_period
    print "np.mean(sm_gyro_time - mcu_gyro_time)", np.mean(sm_gyro_time - mcu_gyro_time)
    print "sm_gyro_time[0]    ", sm_gyro_time[0]
    print "sm_gyro_time[-1]   ", sm_gyro_time[-1]
    print "mcu_gyro_time[0]   ", mcu_gyro_time[0]
    print "mcu_desired_ts     ", mcu_desired_ts

    with open("out/" + time.strftime("%b_%d_%Y_%H_%M_%S") + ".txt", "w+") as out:
            out.writelines('comp_delay2,sm_remote_ts,mcu_desired_ts,sm_mcu_clock_offset\n' + \
                str(comp_delay2) + ',' + str(sm_remote_ts) + ',' + str(mcu_desired_ts) + ',' + str(sm_mcu_clock_offset) + \
                '\n'
    )
    '''

    # Phase alignment
    align_camera_process = subprocess.Popen(("rosrun mcu_interface align_mcu_cam_phase_client " + str(mcu_desired_ts)).split())

    # Some time needed to get camrera frame data by mcu.cpp
    time.sleep(0.1)

    # Send publish_s10_timestamp message to mcu.cpp
    send_offset_subprocess = subprocess.Popen(("rosrun mcu_interface publish_s10_to_mcu_offset_client " + str(sm_mcu_clock_offset)).split())
    
    ### Add record 


    print('Recording started.\nPress Ctrl+C to stop and exit')

    while stop_flag == 0:
        time.sleep(0.01);

    remote.stop_video()
    remote.close()
    depth_cam_listener.unregister()
if __name__ == '__main__':
    main()
