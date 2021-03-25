from pymavlink import mavutil
import csv
import time

def request_message_interval(master : mavutil.mavudp, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        0, 0, 0, 0)

mav_connection = mavutil.mavlink_connection('udpout:raspberrypi.local:14550', )
# the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

print("Connected! Waiting for heartbeat..")

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mav_connection.mav.ping_send(222,0,0,0)
mav_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mav_connection.target_system, mav_connection.target_system))

print("Requesting 200 Hz")
request_message_interval(mav_connection, 129, 200.0)

t0 = time.time()

with open("mav_imu.csv", 'w') as f:
    writer = csv.writer(f)
    writer.writerow([
        "Timestamp (s)",
        "Mav Time (ms)",
        "xgyro (mrad/s)",
        "ygyro (mrad/s)",
        "zgyro (mrad/s)",
        "xacc (mG)",
        "yacc (mG)",
        "zacc (mG)",
    ])
    for _ in range(2000):
        msg = mav_connection.recv_match(type="SCALED_IMU3", blocking=True)
        t = time.time()
        if msg is None:
            print("None msg")
            continue
        else:
            print("Message received.")
        row = [
            t - t0,
            msg.time_boot_ms,
            msg.xgyro,
            msg.ygyro,
            msg.zgyro,
            msg.xacc,
            msg.yacc,
            msg.zacc,
        ]
        writer.writerow(row)