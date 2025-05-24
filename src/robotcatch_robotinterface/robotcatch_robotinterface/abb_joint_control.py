from ABBRobotEGM import EGM
import numpy as np

''' 
core script and egm implementation is from FLo_ABB's git repo
just modified to be a ROS2 node
https://github.com/FLo-ABB/ABB-EGM-Python
'''

def main() -> None:
    """
    Example showing how to make the robot's first joint oscillate between -45 and 45 degrees.
    The robot will complete 4 full movements (4 cycles) and then stop.
    Be sure diEGM is low before running this script.
    Be sure the robot is running before running this script.
    """
    with EGM() as egm:
        angle = 0.0
        amplitude = 45.0     # Maximum joint angle in degrees
        cycles = 0          # Count completed movements
        step = 0.05         # Angular step
        last_target = 0     # 0: start, 1: +45°, -1: -45°

        while True:
            success, state = egm.receive_from_robot()
            if not success:
                print("Failed to receive from robot")
                break

            # Get the current joint angles
            joint_angles = state.joint_angles.copy()
            current_angle = joint_angles[0]

            # Calculate the desired angle for joint 1
            desired_angle = amplitude * np.sin(angle)
            joint_angles[0] = desired_angle

            # Check if we've reached the extremum positions
            if last_target == 0 and abs(desired_angle) >= amplitude * 0.99:
                last_target = 1 if desired_angle > 0 else -1
            elif last_target == 1 and desired_angle <= -amplitude * 0.99:
                last_target = -1
                cycles += 1
                print(f"Completed movement {cycles}")
            elif last_target == -1 and desired_angle >= amplitude * 0.99:
                last_target = 1
                cycles += 1
                print(f"Completed movement {cycles}")

            # Update angle only if we're close to desired position
            if abs(current_angle - desired_angle) < 2.0:  # 2 degree tolerance
                angle += step

            # Send the new joint angles to the robot
            egm.send_to_robot(joint_angles,
                              digital_signal_to_robot=(cycles >= 4))


if __name__ == "__main__":
    main()