from pymavlink import mavutil

connection = mavutil.mavlink_connection(('/dev/tty.usbmodem01'))

# stop other data streams
connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)

# Wait for the first heartbeat to set the system and component ID of remote system for the link
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# param2: 1000000 (Stream interval in microseconds)
message = connection.mav.command_long_encode(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,  # param1: The MAVLink message ID
        1000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )


# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
# Send the COMMAND_LONG
connection.mav.send(message)
message = connection.mav.command_long_encode(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,  # param1: The MAVLink message ID
        500, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )

response = connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
  print("Command accepted")
  print(response)
else:
  print("Command failed")

import time
cnt = 0
start = time.time()
while True:
  cnt += 1
  msg = connection.recv_match(type='SCALED_IMU', blocking=True)
  # print(msg)
  if cnt == 1000:
    end = time.time()
    # cal Hz
    print("Hz: ", cnt / (end - start))
    print(msg)
    cnt = 0
    start = end
