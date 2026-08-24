#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import sys

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    print("\n[SYSTEM] Initializing Navigation Menu...")
    
    # 1. Set Initial Pose (Matches TurtleBot3 default spawn pose in turtlebot3_world: x=-2.0, y=-0.5)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -2.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.orientation.w = 1.0
    
    print("[SYSTEM] Setting Initial Pose to TurtleBot3 Gazebo spawn location (-2.0, -0.5)...")
    nav.setInitialPose(initial_pose)
    
    # 2. Wait for Nav2 to activate
    print("[SYSTEM] Waiting for Nav2 components to wake up...")
    try:
        nav.waitUntilNav2Active(localizer='amcl')
        print("\n[SUCCESS] Nav2 is fully Active!")
    except Exception as e:
        print(f"\n[WARN] Activation check timeout: {e}")
        print("[INFO] You can set 2D Pose Estimate in RViz or proceed directly...")

    try:
        while rclpy.ok():
            print("\n====================================")
            print("   ROBOT NAVIGATION MISSION CONTROL")
            print("====================================")
            print(" 1. Send Goal (Compute Path & Move)")
            print(" 2. Emergency Stop (Cancel Task)")
            print(" 3. Exit Mission Control")
            print("------------------------------------")
            
            choice = input("Enter Choice [1-3]: ").strip()

            if choice == '1':
                try:
                    print("\n--- Enter Target Coordinates ---")
                    x = float(input("Target X: "))
                    y = float(input("Target Y: "))
                    
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.header.stamp = nav.get_clock().now().to_msg()
                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.w = 1.0
                    
                    print(f"\n[MISSION] Pathfinding to ({x}, {y})...")
                    nav.goToPose(goal)
                    
                    # Monitor progress
                    while not nav.isTaskComplete():
                        feedback = nav.getFeedback()
                        if feedback:
                            print(f"Distance to goal: {feedback.distance_remaining:.2f} m", end="\r")
                            sys.stdout.flush()
                        time.sleep(0.5)
                    
                    result = nav.getResult()
                    if result == TaskResult.SUCCEEDED:
                        print("\n[SUCCESS] Goal Reached!")
                    elif result == TaskResult.CANCELED:
                        print("\n[INFO] Mission Canceled.")
                    elif result == TaskResult.FAILED:
                        print("\n[ERROR] Mission Failed.")
                        
                except ValueError:
                    print("\n[ERROR] Invalid input. Please enter numbers.")
                except Exception as e:
                    print(f"\n[ERROR] Mission failed: {e}")

            elif choice == '2':
                nav.cancelTask()
                print("\n[STOP] Emergency Stop Command Sent.")
            
            elif choice == '3':
                print("\n[INFO] Exiting Mission Control.")
                break
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
