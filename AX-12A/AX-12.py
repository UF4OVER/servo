import time
import serial
from serial import SerialException

"""定义伺服电机相关控制寄存器的常量"""
# 一些寄存器不需要写入，只要知道起始位的寄存器就OK
# 识别ID
ID = 0x03
# 波特率设置
BAUDRATE = 0x04
# 反馈延时设置
FB_DELAY = 0x05
# 旋转角度限制
CLOCKWISE_ANGLE_LIMIT = 0x06
COUNTERCLOCKWISE_ANGLE_LIMIT = 0x08
# 温度限制
TEMPERATURE_UPPER_LIMIT = 0x0b
# 电压限制
VOLTAGE_HIGH_LIMIT = 0x0d
VOLTAGE_LOW_LIMIT = 0x0c
# 扭矩设置
MAXIMUM_TORQUE = 0x0e
# 状态反馈
FB_STATUS = 0x10
# LED警告设置
LED_WARNING = 0x11
# 报警控制
DISMISS_TORQUE_ALERT = 0x12
# 扭矩控制
ACTIVATES_TORQUE = 0x18
# LED控制
LED_CONTROL = 0x19
# 运动控制
TARGET_LOCATION = 0x1e  # 目标位置范围 0x00-0x3ff
SPEED = 0x20  # 运动速度范围 0x00-0x3ff
TORQUE_LIMIT = 0x22  # 扭矩限制范围 0x00-0x3ff

"""Instruction"""

PING = 0x01
READ = 0x02
WRITE = 0x03
"""________________________"""
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_RESET = 0x06
INST_SYNC_WRITE = 0x83

try:
    ser = serial.Serial(
        port='COM18',  # 串口号
        baudrate=1000000,  # 波特率
        bytesize=serial.EIGHTBITS,  # 数据位，EIGHTBITS表示8位
        parity=serial.PARITY_NONE,  # 校验位，PARITY_NONE表示无校验
        stopbits=serial.STOPBITS_ONE)  # 停止位，STOPBITS_ONE表示1位停止位
    print(f"成功打开串口")

except SerialException as e:
    print(f"无法打开串口: {e}")


class dynamixel_single:
    """
    单个舵机发送数据
    """

    def __init__(self, uart, id):
        """
        :param uart:串口对象
        :param id:舵机ID
        """
        # uart参数为串口对象，需要自己定义

        self.sum = 0
        self.uart = uart
        self.id = id

    def Servo_Do(self, angle, percent_speed):
        """
        :param angle: 舵机位置
        :param percent_speed: 舵机速度
        :return:
        """

        if not 0 <= percent_speed <= 100:
            raise ValueError("Percent must be between 0.0 and 100.0.")
        # 0-100线性映射百分比到1-0x3ff的范围内
        speed = int(((0x3ff - 1) * percent_speed / 100) + 1)
        # 0-300线性映射百分比到1-0x3ff的范围内
        if not 0 <= angle <= 360:
            raise ValueError("Input value must be between 0 and 300.")
        local = int(((angle / 360) * 1024))
        # 以下列表为舵机位置和速度的写入列表
        # 高位与低位直接在列表中计算
        # 详细的下方有例子
        _list_loca = [TARGET_LOCATION, local & 0xFF, (local >> 8) & 0xFF, speed & 0xFF, (speed >> 8) & 0xFF]
        print(_list_loca)
        print(sum(_list_loca))
        self.sum = sum(_list_loca)
        return _list_loca

    def Angle_Limit(self, limit_cw: int, limit_ccw: int) -> list:
        """
        :param limit_cw:最低角度
        :param limit_ccw:最高角度
        """
        # 线性拟合
        if not 0 <= limit_ccw <= 300:  # 判断输入值是否在0-300之间
            raise ValueError("Input value must be between 0 and 300.")
        local_ccw = int(((limit_ccw / 300) * 1023))
        low_local_ccw = local_ccw & 0xFF  # 高低位
        high_local_ccw = (local_ccw >> 8) & 0xFF
        # 线性拟合
        if not 0 <= limit_cw <= 300:  # 判断输入值是否在0-300之间
            raise ValueError("Input value must be between 0 and 300.")
        local_cw = int(((limit_cw / 300) * 1024))
        low_local_cw = local_cw & 0xFF  # 高低位
        high_local_cw = (local_cw >> 8) & 0xFF

        _list = [CLOCKWISE_ANGLE_LIMIT, low_local_cw, high_local_cw, low_local_ccw, high_local_ccw]
        self.sum = sum(_list)
        return _list

    def Voltage_Limit(self, voltage_low: int, voltage_high: int) -> list:
        if not 0 <= voltage_low <= 300:
            raise ValueError("Input value must be between 0 and 300.")
        l_voltage = int((voltage_low * 10))  # 电压计算
        h_voltage = int((voltage_high * 10))
        _list = [VOLTAGE_LOW_LIMIT, l_voltage, h_voltage]
        self.sum = sum(_list)
        return _list

    def Max_Torque(self, torque_limit: int):
        """
        :param torque_limit:扭矩限制
        :return:
        """
        if not 0 <= torque_limit <= 1023:
            raise ValueError("Input value must be between 0 and 1023.")
        _list = [MAXIMUM_TORQUE, torque_limit & 0xFF, (torque_limit >> 8) & 0xFF]
        self.sum = sum(_list)
        return _list

    def Temperature_Limit(self, temperature_limit: int):
        """
        :param temperature_limit:
        :return:好像没用
        """
        if not 0 <= temperature_limit <= 100:
            raise ValueError("Input value must be between 0 and 100.")
        _list = [TEMPERATURE_UPPER_LIMIT, temperature_limit]
        self.sum = sum(_list)
        return _list

    def Activate_Torque(self, torque_on):
        """
        激活扭矩的同时激活LED
        :param torque_on:1激活，0不激活
        """

        if torque_on:
            _list = [ACTIVATES_TORQUE, 1, 1]
        else:
            _list = [ACTIVATES_TORQUE, 0, 0]
        self.sum = sum(_list)
        return _list

    def change_id(self, new_id):
        """
        修改舵机ID,改了之后不需要再次初始化
        :param new_id:新ID
        """

        if not 0 <= new_id <= 254:
            raise ValueError("Input value must be between 0 and 254.")
        self.id = new_id
        _list = [ID, new_id]
        self.sum = sum(_list)
        return _list

    def NoStatusReturn(self):
        """
        关闭状态反馈
        """
        _list = [FB_STATUS, 0x00]
        self.sum = sum(_list)
        return _list

    # def Read_VoltageAndTemperature(self):
    #     """
    #     读取电压和温度
    #     """
    #     _list = [VOLTAGE_HIGH_LIMIT, 0x2a, 0x02]
    #     self.sum = sum(_list)
    #     return _list

    def receive_packet(self, num: int = 4):  # 接受数据
        rx_value = self.uart.readline()  # 读取串口数据
        if rx_value:  # 判断是否接收到数据
            target_byte = rx_value[num - 1]  # 获取目标位置
            binary_str = bin(target_byte)[2:]  # 转换为二进制字符串
            bin_str = binary_str.zfill(8)  # 填充为8位
            # 判断错误
            if bin_str[7] == '1':
                print("Input Voltage Error")
            if bin_str[6] == '1':
                print("Angle Limit Error")
            if bin_str[5] == '1':
                print("Overheating Error")
            if bin_str[4] == '1':
                print("Range Error")
            if bin_str[3] == '1':
                print("Checksum Error")
            if bin_str[2] == '1':
                print("Overload Error")
            if bin_str[1] == '1':
                print("Instruction Error")
        else:
            print("No data received")

    def send_packet(self, instruction: int, *parameters: list):
        """
        :param instruction: 指令
        :param parameters: 参数
        :return:
        """
        # 判断指令，加长度
        self._packet = []  # 清空列表，只能加在这里，每次调用清空
        if instruction == PING:  # 判断指令
            _length = 0
        elif instruction == WRITE:
            _length = 2 + len(*parameters)
        elif instruction == READ:
            _length = 4
        elif instruction == INST_REG_WRITE:
            _length = 3 + len(*parameters)
        elif instruction == INST_ACTION:
            _length = 2
        elif instruction == INST_RESET:
            _length = 2
        elif instruction == INST_SYNC_WRITE:  # _length = 2 + len(parameters)
            pass
        else:
            raise ValueError("Instruction must be between 0x01 and 0x06.")

        self._packet.extend([0xff, 0xff, self.id])  # 添加包头
        self._packet.append(_length)  # 添加长度
        self._packet.append(instruction)  # 添加指令
        self._packet.extend(*parameters)  # 添加参数
        # cleaned_parameters = [int(item) if isinstance(item, (int, float)) else 0 for item in parameters]
        # self._checksum = sum(cleaned_parameters)
        # for i in parameters:
        #     self._checksum += int(i,16)
        sum_values = self.id + _length + instruction + self.sum
        checksum = ~sum_values & 0xFF  # 计算校验和
        self._packet.append(checksum)  # 添加校验和
        print(self._packet)
        HEV = bytearray(self._packet)  # 转换为字节数组
        print(HEV)
        self.uart.write(HEV)
        print("数据已发送")  # 解码发送的数据以便打印
        print(self.uart.readline())


# example code
a = dynamixel_single(ser, 1)
a.Servo_Do(150, 50)
a.Activate_Torque(0)
a.send_packet(WRITE, a.Activate_Torque(0))
# a.angle_limit(0, 150)
# a.send_packet(WRITE, a.angle_limit(0, 150))
# a.voltage_limit(8, 12)
# a.send_packet(WRITE, a.voltage_limit(8, 12))
# a = dynamixel_single(1, 1)
# a.change_id(0)
# a.send_packet(WRITE, a.change_id(0))
# a.NoStatusReturn()
# a.send_packet(WRITE, a.NoStatusReturn())
