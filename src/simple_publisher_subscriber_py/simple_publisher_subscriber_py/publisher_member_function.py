import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time  # Para obter o tempo em segundos
import sys  # Para ler argumentos de linha de comando


class MinimalPublisher(Node):

    def __init__(self, exec_time=300, sleep_time=0.25):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # Intervalo entre publicações
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.start_time = time.time()  # Armazena o tempo de início em segundos

        # Definindo os tempos passados como parâmetros
        self.exec_time = exec_time
        self.sleep_time = sleep_time

    def timer_callback(self):
        # Verifica o tempo atual
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Se o tempo decorrido for maior ou igual ao tempo de execução especificado
        if elapsed_time >= self.exec_time:
            self.get_logger().info('Tempo limite atingido, encerrando o nó.')
            # Para evitar que o spin continue após o shutdown
            self.timer.cancel()  # Cancela o timer
            self.destroy_node()  # Destroi o nó
            rclpy.shutdown()  # Finaliza o contexto ROS
            return

        # Publica a mensagem normalmente
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # Pausa de acordo com o tempo de sleep fornecido
        time.sleep(self.sleep_time)


def main(args=None):
    rclpy.init(args=args)

    # Lendo os parâmetros de linha de comando
    if len(sys.argv) >= 3:
        exec_time = float(sys.argv[1])  # Primeiro argumento: Tempo de execução em segundos
        sleep_time = float(sys.argv[2])  # Segundo argumento: Tempo de sleep em segundos
    else:
        exec_time = 300  # Tempo padrão de execução (5 minutos)
        sleep_time = 0.25  # Tempo padrão de sleep (0.25 segundos)

    minimal_publisher = MinimalPublisher(exec_time=exec_time, sleep_time=sleep_time)

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    # Destruir o nó explicitamente (opcional)
    if rclpy.ok():  # Verifica se o contexto ainda está ativo antes de encerrar
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
