###############################################################################
#  Copyright (c) 2016-2023
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#     WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.
###############################################################################

"""Pure Pursuit for Topic Action Server."""
from datetime import datetime
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path

from flex_nav_common.action import FollowTopic
from flex_nav_pure_pursuit.pure_pursuit import PurePursuit


class PurePursuitTopic(PurePursuit):
    """Pure Pursuit for Topic Action Server."""

    def __init__(self):
        super().__init__("pure_pursuit_topic")
        self._action_server = ActionServer(self, FollowTopic,
                                           self._action_name,
                                           execute_callback=self.execute,
                                           callback_group=ReentrantCallbackGroup(),
                                           goal_callback=self.goal_callback,
                                           cancel_callback=self.cancel_callback)
        self._result_type = FollowTopic.Result
        self._feedback_type = FollowTopic.Feedback
        self._current_path = None
        self._latest_path = None

        # rospy.spin()

    async def execute(self, goal_handle):
        """
        Execute goal for the FollowTopicActionServer.

        @param goal_handle The goal to process
        @return goal result
        """
        self.get_logger().info('Executing goal...')
        super().execute(goal_handle)  # Activate markers

        # Wait for the lock after prior goal is properly canceled
        self._is_new_goal = True

        timer = self.create_rate(self._controller_rate.value)
        while self._running and rclpy.ok():
            self.get_logger().warn('Waiting for prior pure pursuit topic goal to exit ...')
            timer.sleep()

        self._is_new_goal = False
        self._running = True

        try:
            self._sub = self.create_subscription(Path, goal_handle.topic.data, self.path_topic_cb, 1)
            self.get_logger().info('Success!')
        except Exception:  # pylint: disable=W0703
            self.get_logger().error('Desired topic does not publish a nav_msgs/Path')
            self.destroy_rate(timer)
            return self.set_failed(goal_handle)

        try:
            self.get_logger().info(f"Attempting to listen to topic: {goal_handle.topic.data} ...")

            # Wait for a path to be published
            while (rclpy.ok() and self._running and self._current_path is None
                    and not self._is_new_goal and not goal_handle.is_cancel_requested()):
                self._current_path = self._latest_path
                timer.sleep()

                # This is where the actual work gets done
            while rclpy.ok() and self._running and not self._is_new_goal and not goal_handle.is_cancel_requested():
                self._start_time = self.get_clock().now().to_msg()
                self._done = False
                self._failed = False

                self._current_path = self._latest_path

                if (self._current_path.poses) < 1:
                    self.get_logger().error("Invalid path with "
                                            f"{len(self._current_path.poses)} points: {self._current_path.header.seq}")
                    self._failed = True
                    return None  # return value set in finally

                self.get_logger().info("Executing new path with "
                                       f"{len(self._current_path.poses)} points: {self._current_path.header.seq}")

                self._indice = self.find_furtherest_target(goal_handle.request.path)
                if self._indice < 0:
                    self.get_logger().error(f"{self.get_name()} Invalid starting index {self._indice} - no valid path points!")
                    self._failed = True
                    return  # return value set in finally

                if not self.get_current_target(self._current_path):
                    # Check for distance to terminal target within lookahead
                    # Get target given valid index
                    self._target = self._current_path.poses[-1]

                    dr2 = ((self._target.point.x - self._location.point.x)**2
                           + (self._target.point.y - self._location.point.y)**2)
                    if dr2 < self._lookahead_distance_squared:
                        # Success!
                        self.get_logger().info('Found terminal point - success!')
                        self._done = True
                        return None  # return value set in finally

                    # Failure
                    self.get_logger().error('Failed to find point along current path - failed!')
                    self._failed = True
                    return None  # return value set in finally

                while (rclpy.ok() and self._running
                       and self._current_path.header.stamp == self._latest_path.header.stamp
                       and not self._is_new_goal
                       and not goal_handle.is_cancel_requested()):

                    # Process each segment in turn until either the end or new path received on topic
                    ret = self.pure_pursuit_control(goal_handle, goal_handle.request.goal_info.path)
                    if ret is not None:
                        return None  # action return value set in finally

                    timer.sleep()

        except Exception as exc:  # pylint: disable=W0703
            self.get_logger().error(f"Exception processing {goal_handle}: {type(exc)}\n  {exc}")
            import traceback  # pylint: disable=C0415
            self.get_logger().error(traceback.format_exc().replace("%", "%%"))
            self._failed = True
        finally:
            self._current_path = None
            self._latest_path = None
            try:
                self.destroy_rate(timer)
            except Exception:  # pylint: disable=W0703
                self.get_logger().info("Issue deleting timer")

            if self._sub:
                try:
                    self._sub.destroy()
                except Exception:  # pylint: disable=W0703
                    self.get_logger().info("Issue deleting path topic subscription")
            self._running = False
            if self._is_new_goal and not self._failed and not self._done:
                return self.set_canceled(goal_handle)  # pylint: disable=W0150
            if self._failed:
                return self.set_failed(goal_handle)  # pylint: disable=W0150
            if self._done:
                return self.set_succeeded(goal_handle)  # pylint: disable=W0150
            raise ValueError("Should have set failed, done, or preempted beore this! ")

    def path_topic_cb(self, data):
        """
        Update the desired path.

        @param data (Path): The Path message to process
        """
        self.get_logger().info(f"    Received a new path with {len(data.poses)} points: {data.header.seq}")
        self._latest_path = data


def main(args=None):
    """Implement PurePursuitTopic."""
    rclpy.init(args=args)
    pure_pursuit_topic_node = PurePursuitTopic()

    executor = MultiThreadedExecutor()
    executor.add_node(pure_pursuit_topic_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt! Shut down!")
    except Exception as exc:
        print(f"Exception in executor! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    try:
        pure_pursuit_topic_node.destroy()
    except Exception as exc:
        print(f"Exception in pure_pursuit_topic_node shutdown! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    print(f"{datetime.now()} - Done with pure_pursuit_topic_node!")
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from rclpy.shutdown for pure_pursuit_topic_node: {type(exc)}\n{exc}")
        print(f"{traceback.format_exc().replace('%', '%%')}")


if __name__ == '__main__':
    main()
