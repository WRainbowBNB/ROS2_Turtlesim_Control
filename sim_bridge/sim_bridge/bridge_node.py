import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist
from rov_msg.msg import Imu, Dvl, Controller as ControllerMsg

class SimPhysicsEngine(Node):
    def __init__(self):
        super().__init__('sim_bridge_node')
        self.noise = Noise()
        self.m = 1.0      #单位：kg
        self.I_z = 0.5    #转动惯量
        self.c = 0.5      #流体阻力系数
        self.c_r = 0.3    #旋转阻力系数
        self.F_max = 10.0 #最大阻力
        self.τ_max = 5.0  #最大扭矩
        self.τ = 0.0      #扭矩
        self.L = 0.1      #力臂
        self.a_y = 0.0    #平动y加速度
        self.a_x = 0.0    #x加速度
        self.F_x = 0.0    #当前x推力
        self.F_y = 0.0    #当前y推力
        self.F1 = 0.0     #左推进器
        self.F2 = 0.0     #右推进器
        self.a_yaw = 0.0  #角加速度
        self.v_x = 0.0    #x轴线速度
        self.v_y = 0.0    #y轴线速度
        self.w = 0.0      #角速度
        self.dvl_t = 0.1  #传感器采样周期
        self.imu_t = 0.01 #传感器采样周期
        self.dvl_timer = self.create_timer(self.dvl_t, self.DVL)
        self.imu_timer = self.create_timer(self.imu_t, self.IMU)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.dvl_pub = self.create_publisher(Dvl, 'dvl', 10)
        self.controller_sub = self.create_subscription(ControllerMsg, 'thrust_cmd', self.controller_callback, 10)
        
    #模拟dvl
    def DVL(self):
        # v_y = self.noise.White_Gaussian_Noise(self.v_y)
        # v_y = self.noise.Gauss_Markov_Process(self.v_y)
        #消息实例化
        dvl_msg = Dvl()
        dvl_msg.v_x = 0.0
        dvl_msg.v_y = self.v_y
        #发送消息
        self.dvl_pub.publish(dvl_msg)

    #模拟imu
    def IMU(self):
        self.F_y = min(self.F_max, max(self.F1 + self.F2, -self.F_max))
        self.a_y = (self.F_y - self.c * self.v_y * abs(self.v_y)) / self.m
        self.a_x = 0.0
        self.v_y += self.a_y * self.imu_t
        self.v_x += self.a_x * self.imu_t
        self.τ = min(self.τ_max, max((self.F1 - self.F2) * self.L, -self.τ_max))
        self.a_yaw = (self.τ - self.c_r * self.w * abs(self.w)) / self.I_z
        self.w += self.a_yaw * self.imu_t
        #发给小乌龟
        v_x = self.noise.Gauss_Markov_Process(self.v_x)
        v_y = self.noise.Gauss_Markov_Process(self.v_y)
        w = self.noise.White_Gaussian_Noise(self.w)
        self.send_to_turtle(v_y, v_x, w)
        #模拟传感器噪声，这个数据发给controller
       
       
        #消息实例化
        imu_msg = Imu()
        imu_msg.a_x = self.a_x
        imu_msg.a_y = self.a_y
        imu_msg.w = self.w
        #向controller发送消息
        self.imu_pub.publish(imu_msg)

    #给小乌龟发速度的方法
    def send_to_turtle(self, v_y, v_x, w):
        """
        v_x 我们看是横向即左右，东西 --- 对应小乌龟是前后emmm
        v_y 我们看是纵向即前后，南北 --- 对应小乌龟是左右ovo
        """
        #实例化Twist消息对象
        cmd_msg = Twist()
        #赋值
        cmd_msg.linear.x = v_y #前后，南北
        cmd_msg.linear.y = v_x #左右，东西
        cmd_msg.linear.z = 0.0 #上下
        
        cmd_msg.angular.x = 0.0 #俯仰角
        cmd_msg.angular.y = 0.0 #翻滚角
        cmd_msg.angular.z = w   #航向角
        #发速度
        self.cmd_vel_pub.publish(cmd_msg)
    
    def controller_callback(self, msg:ControllerMsg):
        self.F1 = msg.f1
        self.F2 = msg.f2
        self.τ = msg.tau

class Noise:
    def __init__(self):
        super().__init__()
        self.drift = 0.0 #高斯-马尔科夫噪声
        self.bias = 0.0 #随机游走
    
    #高斯白噪声，正态分布，用来模拟传感器噪声
    def White_Gaussian_Noise(self, a):
        self.white_noise = random.gauss(0, 0.01)
        return a + self.white_noise

    #高斯-马尔科夫噪声，用来模拟水流或者温漂
    def Gauss_Markov_Process(self, a):
        self.drift = 0.01+ random.gauss(0, 0.1)
        return a + self.drift

    #随机游走，用来模拟imu零偏
    def Random_Walk(self, a):
        self.bias += random.gauss(0, 0.0001)
        return a + self.bias

def main(args = None):
    rclpy.init(args = args)
    node = SimPhysicsEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
