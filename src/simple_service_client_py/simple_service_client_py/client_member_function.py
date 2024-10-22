import sys
import time
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    if len(sys.argv) != 5:
        print("Usage: add_two_ints_client X Y execution_time(s) sleep_interval(s)")
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    execution_time = int(sys.argv[3])  # Tempo total de execução em segundos
    sleep_interval = float(sys.argv[4])   # Intervalo de espera (sleep) em segundos

    minimal_client = MinimalClientAsync()

    # Coleta a data e hora de início
    start_time = time.time()
    duration_limit = execution_time  # Tempo total de execução

    while rclpy.ok():
        # Enviar requisição
        future = minimal_client.send_request(a, b)

        # Loop para verificar o tempo
        while rclpy.ok():
            # Verifica se a requisição foi completada
            if future.done():
                response = future.result()
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (a, b, response.sum))
                break
            
            # Verifica o tempo decorrido
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            if elapsed_time >= duration_limit:
                minimal_client.get_logger().info('Tempo limite alcançado. Finalizando o nó...')
                break
            
            # Atraso de acordo com o intervalo especificado
            time.sleep(sleep_interval)

        if elapsed_time >= duration_limit:
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
