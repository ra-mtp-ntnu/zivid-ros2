from ros2lifecycle.api import call_change_states
from ros2lifecycle.api import call_get_available_transitions
from ros2lifecycle.api import call_get_states
from zivid_interfaces.srv import Capture

import sys
import os
import time

import rclpy 
from rclpy.node import Node

class Request(Node):

    def __init__(self):
        super().__init__('capture_request_node')


        # If zivid_camera isn't running, this will get stuck in an infinite loop.
        states = call_get_states(node=self, node_names=['zivid_camera'])
        print(states)
        state = states['zivid_camera'].label
        
        while True:
            if state == 'active':
                break

            self.get_logger().debug("\nCurrent state: %s" % state)
            transitions = wait_until(call_get_available_transitions, {'node': self, 'states': {'zivid_camera': None}}, {'zivid_camera': []}, 2.0)

            transitions = transitions['zivid_camera']        

            for transition in [t.transition for t in transitions]:
                if transition.label == 'configure' or transition.label == 'activate':
                    break

            self.get_logger().debug("Trying to make %s transition." % transition.label)
            # TODO: Check that the transition is okay. 
            results = call_change_states(node=self, transitions={'zivid_camera': transition})
            
            states = call_get_states(node=self, node_names=['zivid_camera'])
            print(states)
            state = states['zivid_camera'].label

            time.sleep(0.25)

        self.get_logger().info("Camera is active.")


        self.client = self.create_client(Capture, 'capture')
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Capture service not available, waiting again...')
        self.req = Capture.Request()

        self.timer = self.create_timer(5, self.timer_callback)

        self.request_count = 1

    def timer_callback(self):
        self.get_logger().info("Sending capture request #%d" % self.request_count)
        self.future = self.client.call_async(self.req)
        self.request_count += 1
              

def wait_until(function, args, not_value, timeout_sec: float = None):
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        success = False
        while timeout_sec > 0.0:
            if args is None:
                output = function()
            else:
                output = function(**args)
            if output is not not_value:
                success = True
                break 
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

        if success == False:
            raise TimeoutError

        return output

def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)


    requestnode = Request()
    rclpy.spin(requestnode)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    requestnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
