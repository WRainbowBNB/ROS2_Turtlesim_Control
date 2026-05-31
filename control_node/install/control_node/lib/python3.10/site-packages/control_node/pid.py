#此为pid封装，包括位置式和增量式
class PID():
    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.MAXOutput = 0.0
        self.MAXIntegral = 0.0
        #运行时状态
        self.target_value= 0       #目标值
        self.current_value = 0     #当前值
        self.error = [0, 0, 0]     #当前误差
        self.integral_sum = 0      #积分项累加值
        self.output = 0            #PID输出值
        self.PID_DT = 0.01 #PID计算周期，单位: s

    #位置式
    def Position_PID(self, target_value, current_value):
        #赋值
        self.target_value = target_value
        self.current_value = current_value
        #1.计算误差
        self.error[0] = self.target_value - self.current_value
        #2.计算积分项
        self.integral_sum += self.error[0] * self.PID_DT #周期10ms
        #积分限幅
        if self.integral_sum > self.MAXIntegral:
            self.integral_sum = self.MAXIntegral
        elif self.integral_sum < -self.MAXIntegral:
            self.integral_sum = -self.MAXIntegral
        #3.计算微分项
        self.value_kd = (self.error[0] - self.error[1]) / self.PID_DT   #周期10ms
        #4.计算PID输出
        self.output = self.error[0] * self.kp + self.integral_sum * self.ki + self.value_kd * self.kd
        #更新误差
        self.error[1] = self.error[0]
        #5.输出限幅
        if self.output > self.MAXOutput:
            self.output = self.MAXOutput
        elif self.output < -self.MAXOutput:
            self.output = -self.MAXOutput

        return self.output
    
    #增量式
    def Incremental_PID(self, target_value, current_value):
        #赋值
        self.target_value = target_value
        self.current_value = current_value
        #1.计算误差
        self.error[0] = self.target_value - self.current_value
        #2.计算微分项
        self.value_kd = (self.error[0] - 2 * self.error[1] + self.error[2]) / self.PID_DT   #周期10ms
        #4.计算PID输出
        self.output = (self.error[0] - self.error[1]) * self.kp + self.error[0] * self.ki * self.PID_DT + self.value_kd *self.kd
        #更新误差
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        #5.输出限幅
        if self.output > self.MAXOutput:
            self.output = self.MAXOutput
        elif self.output < -self.MAXOutput:
            self.output = -self.MAXOutput

        return self.output
    
    #flag 内环1 外环2 改参数用的
    def Set_Param(self, msg, flag):
        if flag == 1:
            self.kp = msg.pos_kp
            self.ki = msg.pos_ki
            self.kd = msg.pos_kd
        elif flag == 2:
            self.kp = msg.inc_kp
            self.ki = msg.inc_ki
            self.kd = msg.inc_kd
            self.target_value = msg.target_depth #此为目标深度