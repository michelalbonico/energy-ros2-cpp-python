import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys  # Para leitura dos argumentos


class MinimalSubscriber(Node):

    def __init__(self, execution_time, sleep_time):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # evitar o aviso de variável não utilizada

        self.start_time = time.time()  # Armazena o tempo de início
        self.execution_time = execution_time  # Tempo de execução em segundos
        self.sleep_time = sleep_time  # Tempo de sleep entre as mensagens

    def listener_callback(self, msg):
        # Verifica o tempo atual
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Se o tempo decorrido for maior ou igual ao tempo de execução
        if elapsed_time >= self.execution_time:
            self.get_logger().info('Tempo limite atingido, encerrando o nó.')
            # Cancela a assinatura e destrói o nó
            self.destroy_node()
            rclpy.shutdown()  # Finaliza o contexto ROS
            return

        # Continua normalmente recebendo as mensagens
        self.get_logger().info('I heard: "%s"' % msg.data)

        # Pausa de acordo com o valor de sleep_time
        time.sleep(self.sleep_time)


def main(args=None):
    rclpy.init(args=args)

    # Verifica e lê os argumentos passados via linha de comando
    if len(sys.argv) < 3:
        print("Uso: minimal_subscriber.py <tempo_execucao> <tempo_sleep>")
        print("Exemplo: minimal_subscriber.py 300 0.25")
        return

    # Obtém os parâmetros a partir dos argumentos de linha de comando
    execution_time = float(sys.argv[1])  # Tempo total de execução (em segundos)
    sleep_time = float(sys.argv[2])  # Tempo de sleep entre as mensagens (em segundos)

    # Cria o nó com os parâmetros fornecidos
    minimal_subscriber = MinimalSubscriber(execution_time, sleep_time)

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    # Verifica se o contexto ainda está ativo antes de tentar destruir o nó ou encerrar
    if rclpy.ok():
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
