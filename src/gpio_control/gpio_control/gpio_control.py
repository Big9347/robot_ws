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
        self.motor_driver_power = Button(18,pull_up=False)
        self.stop_button =Button(17,pull_up=False)
        self.buzzer = Buzzer(1)
        self.lift_relay_forward = OutputDevice(4,active_high=True,initial_value=False)
        self.lift_relay_reverse = OutputDevice(5,active_high=True,initial_value=False)
      
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
        self.lidar_motor_start = self.create_client(Empty, 'start_motor')
        if not self.lidar_motor_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: start_motor service not available')

        self.lidar_motor_stop = self.create_client(Empty, 'stop_motor')
        if not self.lidar_motor_stop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WARNING: stop_motor service not available')
        
        self.lidar_motor_req = Empty.Request()

        self.client_futures = []
    def send_start_lidar_motor_req(self):
        if self.lidar_motor_start.service_is_ready():
            print("Starting lidar...")
            self.client_futures.append(self.lidar_motor_start.call_async(self.lidar_motor_req))
        else:
            print("start_motor not ready!")

    def send_stop_lidar_motor_req(self):
        if self.lidar_motor_stop.service_is_ready():
            print("Stopping lidar...")
            self.client_futures.append(self.lidar_motor_stop.call_async(self.lidar_motor_req))
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
        self.send_stop_lidar_motor_req()
        self.buzzer.on()
    
def main(args=None):
    
    rclpy.init(args=args)

    gpio_control = GpioControl()

    

    # rate = face_player.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(gpio_control)
        gpio_control.check_for_finished_calls()
        if(gpio_control.motor_driver_power.is_pressed):
            gpio_control.get_logger().info("Motor power is cut")
            gpio_control.stanby_state()
        elif (gpio_control.stop_button.is_pressed):
            gpio_control.get_logger().info("Stop button is pressed")
            gpio_control.stanby_state()
            time.sleep(2)
        else:
            gpio_control.buzzer.off()
        time.sleep(0.01)
        
        

    gpio_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
