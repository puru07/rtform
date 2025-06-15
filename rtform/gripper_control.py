import socket
import time

def send_urscript(robot_ip, script):
    port = 30002  # UR secondary interface port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((robot_ip, port))
    s.send((script + '\n').encode('utf8'))
    time.sleep(0.5)
    s.close()

robot_ip = "172.22.22.2"  # ⚡ Change this if your UR5e IP is different

def activate_gripper():
    script = """
set_tool_voltage(24)          # Set tool output voltage to 24V
set_tool_digital_out(0, True) # Enable RS485
sleep(0.5)
write_output_integer_register(0, 9)  # Activate gripper (register 0)
"""
    send_urscript(robot_ip, script)

def open_gripper():
    script = """
set_tool_digital_out(0, True)
sleep(0.2)
write_output_integer_register(1, 0)  # Open gripper (register 1)
"""
    send_urscript(robot_ip, script)

def close_gripper():
    script = """
set_tool_digital_out(0, True)
sleep(0.2)
write_output_integer_register(1, 255)  # Close gripper (register 1)
"""
    send_urscript(robot_ip, script)

# Run this sequence
print("Activating gripper...")
activate_gripper()
time.sleep(2)

print("Opening gripper...")
open_gripper()
time.sleep(2)

print("Closing gripper...")
close_gripper()
time.sleep(2)

print("Done!")
