from common.ros_robot_controller_sdk import Board

print("Creating Board...")
board = Board()
print("Board initialized successfully")

board.pwm_servo_set_position(1.0, [
    [1, 1500],
    [3, 1700],
    [4, 1300],
    [5, 1600],
])