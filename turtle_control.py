import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose
import math

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')

        # Inicialização dos publishers, subscribers e variáveis
        self.init_publisher()
        self.init_subscribers()
        self.init_variables()

        # Inicializar o timer para chamar o pub_callback periodicamente
        self.timer = self.create_timer(0.1, self.pub_callback)

    def init_publisher(self):
        self.velocity_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.goal_subscriber = self.create_subscription(Pose2D, 'diego/goal', self.goal_callback, 10)

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.k_omega = 1.0
        self.v_max = 1.0
        self.distance_tolerance = 0.1  # Tolerância para parar a tartaruga

    def pose_callback(self, data):
        self.x = round(data.x, 2)
        self.y = round(data.y, 2)
        self.theta = round(data.theta, 2)

    def goal_callback(self, data):
        self.x_goal = data.x
        self.y_goal = data.y

    def normalize_angle(self, angle):
        # Normaliza o ângulo para o intervalo [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def pub_callback(self):
        # Cálculo do erro de posição
        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y

        # Cálculo da distância euclidiana até o ponto desejado
        rho = math.sqrt(x_error ** 2 + y_error ** 2)

        # Verifica se a tartaruga está dentro da tolerância de erro
        if rho < self.distance_tolerance:
            # Se estiver dentro da tolerância, para a tartaruga
            v = 0.0
            omega = 0.0
        else:
            # Cálculo do ângulo entre a orientação atual do robô e o vetor rho
            alpha = math.atan2(y_error, x_error) - self.theta
            alpha = self.normalize_angle(alpha)

            # Controle de velocidade linear e angular
            v = self.v_max * math.tanh(rho)
            omega = self.k_omega * alpha

        # Publicação da mensagem de velocidade
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.velocity_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_control = TurtleControl()
    rclpy.spin(turtle_control)
    turtle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

