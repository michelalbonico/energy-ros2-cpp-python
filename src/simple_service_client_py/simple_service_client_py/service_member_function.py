from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
import time
import sys

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    if len(sys.argv) != 3:
        print("Usage: add_two_ints_service execution_time(s) sleep_interval(s)")
        return

    execution_time = int(sys.argv[1])  # Tempo total de execução em segundos
    sleep_interval = float(sys.argv[2])   # Intervalo de espera (sleep) em segundos

    # Coleta a data e hora de início
    start_time = time.time()
    duration_limit = execution_time  # Tempo total de execução

    while rclpy.ok():
        rclpy.spin_once(minimal_service)

        # Verifica o tempo decorrido
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        if elapsed_time >= duration_limit:
            minimal_service.get_logger().info('Tempo limite alcançado. Finalizando o nó...')
            break
        
        # Atraso de acordo com o intervalo especificado
        time.sleep(sleep_interval)

    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
