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

"""Pure pursuit algorithm implementing FollowPath action."""
from datetime import datetime
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from flex_nav_common.action import FollowPath
from flex_nav_pure_pursuit.pure_pursuit import PurePursuit


class PurePursuitPath(PurePursuit):
    """Pure Pursuit Path follower."""

    def __init__(self):
        super().__init__("pure_pursuit_path")
        self._action_server = ActionServer(self, FollowPath,
                                           self._action_name,
                                           execute_callback=self.execute,
                                           callback_group=ReentrantCallbackGroup(),
                                           goal_callback=self.goal_callback,
                                           cancel_callback=self.cancel_callback)

        self._result_type = FollowPath.Result
        self._feedback_type = FollowPath.Feedback

    async def execute(self, goal_handle):
        """
        Execute the goal for the FollowPathActionServer.

        @param goal_handle The goal to process
        @return goal result
        """
        self.get_logger().info('Executing goal...')
        super().execute(goal_handle)  # Activate markers

        timer = self.create_rate(self._controller_rate.value)
        try:
            # Wait for the lock after prior goal is properly canceled
            self._is_new_goal = True

            while self._running and rclpy.ok():
                self.get_logger().warn(0.25, f"{self.get_name()} Waiting for prior pure pursuit goal to exit ...")

            self._is_new_goal = False
            self._running = True

            # Depends only on tf transforms now
            # if not self._last_odom_msg:
            #     self.get_logger().error(f"{self.get_name()} No odometry message received")
            #     self._failed = True
            #     return None  # return value set in finally

            self._start_time = self.get_clock().now()
            self._done = False
            self._failed = False

            self._indice = self.find_furtherest_target(goal_handle.request.path)
            if self._indice < 0:
                self.get_logger().error(f"{self.get_name()} Invalid starting index {self._indice} - no valid path points!")
                self._failed = True
                return  # return value set in finally

            while self._running and rclpy.ok() and not self._is_new_goal and not goal_handle.is_cancel_requested:
                ret = self.pure_pursuit_control(goal_handle, goal_handle.request.path)
                if ret is not None:
                    self.get_logger().warn(f"{self.get_name()} terminating pure pursuit loop ...")
                    return None  # return value set in finally

                timer.sleep()
        except Exception as exc:  # pylint: disable=W0703
            self.get_logger().error(f"Exception processing {goal_handle}: {type(exc)}\n  {exc}")
            import traceback  # pylint: disable=C0415
            self.get_logger().error(traceback.format_exc().replace("%", "%%"))
            self._failed = True
        finally:
            try:
                self.destroy_rate(timer)
            except Exception:  # pylint: disable=W0703
                self.get_logger().info("Issue deleting timer")

            self._running = False
            if (self._is_new_goal or goal_handle.is_cancel_requested) and not self._failed and not self._done:
                return self.set_canceled(goal_handle)  # pylint: disable=W0150
            if self._failed:
                return self.set_failed(goal_handle)  # pylint: disable=W0150
            if self._done:
                return self.set_succeeded(goal_handle)  # pylint: disable=W0150
            raise ValueError("Should have set failed, done, or preempted beore this! ")


def main(args=None):
    """Implement PurePursuitPath follower."""
    rclpy.init(args=args)
    pure_pursuit_path_node = PurePursuitPath()

    executor = MultiThreadedExecutor()
    executor.add_node(pure_pursuit_path_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt! Shut down!")
    except Exception as exc:
        print(f"Exception in executor! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    try:
        pure_pursuit_path_node.destroy()
    except Exception as exc:
        print(f"Exception in pure_pursuit_path_node shutdown! {type(exc)}\n  {exc}")
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}")

    print(f"{datetime.now()} - Done with pure_pursuit_path_node!")
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from rclpy.shutdown for pure_pursuit_path_node: {type(exc)}\n{exc}")
        print(f"{traceback.format_exc().replace('%', '%%')}")


if __name__ == '__main__':
    main()
