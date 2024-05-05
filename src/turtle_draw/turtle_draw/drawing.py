import rclpy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from turtlesim.srv import Kill
from rclpy.node import Node
import time

class PublisherController(Node):
    def __init__(self, turtle_name):
        super().__init__('publisher_controller' + turtle_name)
        self.publisher = self.create_publisher(Twist, f'{turtle_name}/cmd_vel', 10)

    def turtle_move(self, linear_speed, angular_speed, duration):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = (angular_speed * 3.14159265359/180)

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            self.get_logger().info('Movimento enviado: linear=%f, angular=%f' % (linear_speed, angular_speed))
            time.sleep(0.1)

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')

    def spawn_turtle(self, x_position, y_position, theta):
        self.get_logger().info('Iniciando spawn_turtle()')

        # Criando o cliente para o serviço de spawn
        client = self.create_client(Spawn, 'spawn')

        # Aguardando até que o serviço esteja disponível
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço de spawn não disponível.')
            return

        # Criando a requisição para o serviço de spawn
        request = Spawn.Request()
        request.x = x_position
        request.y = y_position
        request.theta = (theta * 3.14159265359/180)

        # Chamando o serviço de spawn
        self.get_logger().info('Fazendo chamada para o serviço de spawn...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Chamada para o serviço de spawn concluída com sucesso.')
        else:
            self.get_logger().info('Falha ao chamar o serviço de spawn.')
    
    def set_turtle_pen(self, turtle_name, r, g, b):
        client = self.create_client(SetPen, f'{turtle_name}/set_pen')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço não disponível, esperando...')

        request = SetPen.Request()
        request.r = r  
        request.g = g   
        request.b = b   
        request.width = 8  

        self.get_logger().info('Fazendo chamada para o serviço set_pen...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Chamada para o serviço set_pen concluída.')

    def kill_turtle(self, turtle_name):
        client = self.create_client(Kill, 'kill')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço não disponível, esperando...')

        request = Kill.Request()
        request.name = turtle_name

        self.get_logger().info(f'Fazendo chamada para o serviço kill para a tartaruga {turtle_name}...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Chamada para o serviço kill para a tartaruga {turtle_name} concluída.')

def heart_draw(turtle_name):
    rclpy.init()
    controller = PublisherController(turtle_name=turtle_name)

    controller.turtle_move(0.0, 90, 0.1)
    rclpy.spin_once(controller, timeout_sec=1.0)

    controller.turtle_move(4.0, 180, 0.6)
    rclpy.spin_once(controller, timeout_sec=0.6)

    controller.turtle_move(4.0, 0, 0.4)
    rclpy.spin_once(controller, timeout_sec=0.4)

    controller.turtle_move(0.0, 90, 0.1)
    rclpy.spin_once(controller, timeout_sec=1.0)

    controller.turtle_move(4.0, 0, 0.4)
    rclpy.spin_once(controller, timeout_sec=0.4)

    controller.turtle_move(4.0, 180, 0.6)
    rclpy.spin_once(controller, timeout_sec=0.6)

    controller.turtle_move(0.0, 0, 0.4)
    rclpy.spin_once(controller, timeout_sec=0.4)

    rclpy.shutdown()

def spawn():
    rclpy.init()
    client = ServiceClient()
    client.spawn_turtle(6.0, 6.0, 90.0)
    client.spawn_turtle(4.2, 6.8, 115.0)
    client.spawn_turtle(3.0, 5.0, 150.0)
    client.spawn_turtle(4.2, 3.0, 200.0)
    client.spawn_turtle(6.6, 3.3, 300.0)
    client.spawn_turtle(8.2, 4.3, -15.0)
    client.spawn_turtle(8.0, 6.0, 45.0)
    
    rclpy.shutdown()

def set_pen(turtle_name, r, g, b):
    rclpy.init()
    client = ServiceClient()
    client.set_turtle_pen(turtle_name=turtle_name, r=r, g=g, b=b)
    rclpy.shutdown()

def kill(turtle_name):
    rclpy.init()
    client = ServiceClient()
    client.kill_turtle(turtle_name=turtle_name)
    rclpy.shutdown()

def go_head(turtle_name):
    rclpy.init()
    controller = PublisherController(turtle_name=turtle_name)
    controller.turtle_move(10.0, 0, 0.2)
    rclpy.spin_once(controller, timeout_sec=0.2)
    rclpy.shutdown()

def main():
    set_pen('turtle1', 255, 0 ,0)
    heart_draw('turtle1')
    kill('turtle1')
    spawn()

    set_pen('turtle2', 255, 255 ,0)
    set_pen('turtle3', 255, 255 ,0)
    set_pen('turtle4', 255, 255 ,0)
    set_pen('turtle5', 255, 255 ,0)
    set_pen('turtle6', 255, 255 ,0)
    set_pen('turtle7', 255, 255 ,0)
    set_pen('turtle8', 255, 255 ,0)

    go_head('turtle1')
    go_head('turtle2')
    go_head('turtle3')
    go_head('turtle4')
    go_head('turtle5')
    go_head('turtle6')
    go_head('turtle7')
    go_head('turtle8')

    kill('turtle1')
    kill('turtle2')
    kill('turtle3')
    kill('turtle4')
    kill('turtle5')
    kill('turtle6')
    kill('turtle7')
    kill('turtle8')


    

if __name__ == '__main__':
    main()

