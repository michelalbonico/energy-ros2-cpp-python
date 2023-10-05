import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import pyRAPL

# pyRAPL.setup()

#csv_output = pyRAPL.outputs.CSVOutput('result.csv')
#meter = pyRAPL.Measurement('in-main')

class ScriptTerminationException(Exception):
    pass

class MinimalPublisher(Node):

    #@pyRAPL.measureit(output=csv_output)
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    #@pyRAPL.measureit(output=csv_output)
    def timer_callback(self):
        if self.i < 100:
            msg = String()
            msg.data = 'Hello, world! %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1
        else:
            self.timer.cancel()
            raise ScriptTerminationException("Script has completed its task.")
        
#@pyRAPL.measureit(output=csv_output)
def main(args=None):
    #meter.begin()
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except ScriptTerminationException as e:
        minimal_publisher.get_logger().info("Script terminated.")
    finally:
        # Destroy the node and shutdown ROS
        minimal_publisher.destroy_node()
        rclpy.shutdown()
    #meter.end()
    #meter.export(csv_output)


if __name__ == '__main__':
    main()
    #csv_output.save()