import time
from src.RemoteControl import RemoteControl
from concurrent.futures import ThreadPoolExecutor
import subprocess
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import pandas as pd
from io import StringIO
from src.TimeSync import TimeSync2

import matplotlib.pyplot as plt

HOST = '10.30.65.160'  # The smartphone's IP address

mcu_imu_time = []
mcu_imu_data = []


def callback(data):
    dat = data.header.stamp.secs + data.header.stamp.nsecs  / 1e9
    mcu_imu_time.append(dat)

    dat = data.angular_velocity
    mcu_imu_data.append([dat.x, dat.y, dat.z])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("mcu_imu", Imu, callback)
    #rospy.spin()

def main():
    remote = RemoteControl(HOST)
    start_duration = 1
    main_duration = 4
    end_duration = 5

    with ThreadPoolExecutor(max_workers=1) as executor:
        print('imus gathering started')
        future = executor.submit(remote.get_imu, 1000 * (start_duration + main_duration + end_duration), False, True)
        listener()
        time.sleep(start_duration)
        print('start shaking')
        time.sleep(main_duration)
        print('put back')
        time.sleep(end_duration)
        rospy.signal_shutdown('it is enough')
        accel_data, gyro_data = future.result()
        print('stopped')


    mcu_gyro_data = np.array(mcu_imu_data) - np.array(mcu_imu_data)[:200].mean(axis=0)
    mcu_gyro_time = np.array(mcu_imu_time)
    print(gyro_data[:200])

    sm_df = pd.read_csv(StringIO(unicode(gyro_data)), header=None, index_col=False)
    sm_gyro_data = sm_df.iloc[1:, :3].to_numpy()
    sm_gyro_time = sm_df.iloc[1:, 3].to_numpy() / 1e9
    
    print(mcu_gyro_data.shape, sm_gyro_data.shape, mcu_gyro_time.shape, sm_gyro_time.shape)

    min_length = min(sm_gyro_time.shape[0], mcu_gyro_time.shape[0])


    mcu_gyro_data, mcu_gyro_time, sm_gyro_data, sm_gyro_time = \
    mcu_gyro_data[:min_length], mcu_gyro_time[:min_length], \
    sm_gyro_data[:min_length], sm_gyro_time[:min_length]

    time_sync2 = TimeSync2(
        mcu_gyro_data, sm_gyro_data, mcu_gyro_time, sm_gyro_time, False
    )
    time_sync2.resample(accuracy=1)
    time_sync2.obtain_delay()
    comp_delay2 = time_sync2.time_delay

    bashCommand = "rosrun mcu_interface start_mcu_cam_trigger_client"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    #TODO needed some time

    #resulting_offset_s = np.mean(sm_gyro_time - mcu_gyro_time) + comp_delay2    #resulting_offset_s = (sm_gyro_time[0] - mcu_gyro_time[0] + comp_delay2)
    mcu_timestamp = np.mean(mcu_gyro_time) + comp_delay2
    s10_timestamp = np.mean(sm_gyro_time)
    resulting_offset_s = mcu_timestamp - s10_timestamp

    bashCommand = "rosrun mcu_interface publish_s10_timestamp_client " + str(mcu_timestamp) + " " + str(s10_timestamp)
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    phase_ns, duration_ns = remote.start_video()
    phase_in = phase_ns / 1e9
    phase_out = phase_in - resulting_offset_s

    print ("comp_delay2, phase_in, phase_out, resulting_offset_s, duration_ns")
    print (comp_delay2, phase_in, phase_out, resulting_offset_s, duration_ns)
    with open(time.strftime("%b_%d_%Y_%H_%M_%S") + ".txt", "w+") as out:
            out.writelines('comp_delay2,phase_in,phase_out,resulting_offset_s\n' + \
                str(comp_delay2) + ',' + str(phase_in) + ',' + str(phase_out) + ',' + str(resulting_offset_s) + \
                '\n'
    )
    #print(resulting_offset_s, sm_gyro_time[0] - mcu_gyro_time[0], mcu_gyro_data.shape, sm_gyro_data.shape, mcu_gyro_time.shape, sm_gyro_time.shape)

    time.sleep(1)

    bashCommand = "rosrun mcu_interface align_mcu_cam_phase_client " + str(phase_out)
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    
    time.sleep(1)

    remote.stop_video()
    #print('EXITED')
    remote.close()
    
    plt.plot(mcu_gyro_time, np.mean(mcu_gyro_data, axis=1))
    plt.plot(sm_gyro_time - resulting_offset_s, np.mean(sm_gyro_data, axis=1), '--')

    plt.show()

if __name__ == '__main__':
    main()
