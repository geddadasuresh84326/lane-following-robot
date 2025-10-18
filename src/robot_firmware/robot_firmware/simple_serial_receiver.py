#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialReceiver(Node):

    def __init__(self):
        super().__init__("simple_serial_publisher")
        self.pub_ = self.create_publisher(String,"serial_receiver",10)
        self.declare_parameter("port","/dev/ttyACM0")
        self.declare_parameter("baudrate",115200)
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter(f"baudrate").value
        self.serial_conn_ = serial.Serial(port=self.port_, baudrate=self.baudrate_)

        self.timer_ = self.create_timer(0.01,self.timerCallback)

    def timerCallback(self):
        if rclpy.ok() and self.serial_conn_.is_open:
            data = self.serial_conn_.readline()
            try:
                data.decode("utf-8")
            except:
                return
            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)
        
def main():
    rclpy.init()

    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()