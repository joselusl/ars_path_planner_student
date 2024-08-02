#!/usr/bin/env python3

import rclpy

from ars_path_planner_src.ars_path_planner_manager_ros import ArsPathPlannerRos


def main(args=None):

  rclpy.init(args=args)

  ars_path_planner_ros = ArsPathPlannerRos()

  ars_path_planner_ros.open()

  try:
      ars_path_planner_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_path_planner_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()
