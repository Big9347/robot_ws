from gpiozero import Button
from gpiozero import LED
from gpiozero import Buzzer
from gpiozero import OutputDevice
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import time
from std_srvs.srv import Empty

class GpioControl(Node):
    def __init__(self):
        super().__init__('gpio_control')
        self.get_logger().info('Setting up GPIO...')

        # Configuration
        self.start_button = Button(18,pull_up=False)
        self.bumper_right = Button(18,pull_up=False)
        self.bumper_left = Button(18,pull_up=False)
      
        self.stop_button_pin =Button(17,pull_up=False)
        self.buzzer_pin = Buzzer(1)
        self.lift_relay_forward = OutputDevice(4,active_high=True,initial_value=False)
        self.lift_relay_reverse = OutputDevice(5,active_high=True,initial_value=False)
        self.operation_lamp_relay = OutputDevice(17,active_high=True,initial_value=False)

        # ROS Interactions

        self.range_sub = self.create_subscription(
            Range,
            'range_publisher',
            self.range_callback,
            10)
        self.range_sub  # prevent unused variable warning
        self.batt_vol = self.create_subscription(
            Float64,
            'battery_state',
            self.battery_callback,
            10)
        self.motor_start = self.create_client(Empty, 'start_motor')
        if not self.motor_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: start_motor service not available')

        self.motor_stop = self.create_client(Empty, 'stop_motor')
        if not self.motor_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: stop_motor service not available')
        
        self.motor_req = Empty.Request()

        self.client_futures = []
    def send_start_motor_req(self):
        if self.motor_start.service_is_ready():
            print("Starting lidar...")
            self.client_futures.append(self.motor_start.call_async(self.motor_req))
        else:
            print("start_motor not ready!")

    def send_stop_motor_req(self):
        if self.motor_stop.service_is_ready():
            print("Stopping lidar...")
            self.client_futures.append(self.motor_stop.call_async(self.motor_req))
        else:
            print("stop_motor not ready!")

    def range_callback(self, range_vals):
        self.get_logger().info(range_vals.range)
        return
    def check_for_finished_calls(self):
        incomplete_futures = []
        for f in self.client_futures:
            if f.done():
                res = f.result()
            else:
                incomplete_futures.append(f)  
    def lift_up(self):
        if(self.lift_relay_forward.value and not self.lift_relay_reverse.value):
            self.get_logger().info("Already lifted")
        else:
            self.get_logger().info("lifting...")
            self.lift_relay_forward.on()
            self.lift_relay_reverse.off()
    def lift_down(self):
        if(self.lift_relay_reverse.value and not self.lift_relay_forwardvalue):
            self.get_logger().info("Already dropped")
        else:
            self.get_logger().info("Dropping...")
            self.lift_relay_reverse.on()
            self.lift_relay_forward.off()
    def lift_stop(self):
        self.get_logger().info("Lift cut off...")
        self.lift_relay_reverse.off()
        self.lift_relay_forward.off()
    def stanby_state(self):
        self.lift_stop()
        self.send_stop_motor_req()

    
def main(args=None):
    
    rclpy.init(args=args)

    gpio_control = GpioControl()

    gpio_control.stanby_state()

    # rate = face_player.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(gpio_control)
        gpio_control.check_for_finished_calls()
        time.sleep(0.01)
        
        

    gpio_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
