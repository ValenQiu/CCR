# -*- coding: utf-8 -*-
import time

class MotorCommander:
    # Commands
    CMD_SERVO_FRAME_HEADER = 0x55
    CMD_SERVO_MOVE = 3
    CMD_SERVO_MOVE_TIME_WRITE = 1
    CMD_SERVO_MOVE_TIME_READ = 2
    CMD_SERVO_MOVE_TIME_WAIT_WRITE = 7
    CMD_SERVO_MOVE_TIME_WAIT_READ = 8
    CMD_SERVO_MOVE_START = 11
    CMD_SERVO_MOVE_STOP = 12
    CMD_hand_id_WRITE = 13
    CMD_hand_id_READ = 14
    CMD_SERVO_ANGLE_OFFSET_ADJUST = 17
    CMD_SERVO_ANGLE_OFFSET_WRITE = 18
    CMD_SERVO_ANGLE_OFFSET_READ = 19
    CMD_SERVO_ANGLE_LIMIT_WRITE = 20
    CMD_SERVO_ANGLE_LIMIT_READ = 21
    CMD_SERVO_VIN_LIMIT_WRITE = 22
    CMD_SERVO_VIN_LIMIT_READ = 23
    CMD_SERVO_TEMP_MAX_LIMIT_WRITE = 24
    CMD_SERVO_TEMP_MAX_LIMIT_READ = 25
    CMD_SERVO_TEMP_READ = 26
    CMD_SERVO_VIN_READ = 27
    CMD_SERVO_POS_READ = 0x15
    CMD_SERVO_OR_MOTOR_MODE_WRITE = 29
    CMD_SERVO_OR_MOTOR_MODE_READ = 30
    CMD_SERVO_LOAD_OR_UNLOAD_WRITE = 31
    CMD_SERVO_LOAD_OR_UNLOAD_READ = 32
    CMD_SERVO_LED_CTRL_WRITE = 33
    CMD_SERVO_LED_CTRL_READ = 34
    CMD_SERVO_LED_ERROR_WRITE = 35
    CMD_SERVO_LED_ERROR_READ = 36

    '''
    Communication protocol:
    -------------------------------------------------------------------------------
    |   Header  | Motor ID | Data Length | Command |  Parameters   | Check Sum |
    |-----------------------------------------------------------------------------|
    | 0x55 0x55 |    ID    |   length    |   cmd   | prm1 ... prmN |  checksum |
    -------------------------------------------------------------------------------
    where:
    Header: two 0x55, means the message is sended
    ID: the ID of each motor, range from 0 to 253, in hexadecimal number (0x00 to 0xFD).
        Boardcast ID is 254(0xEE), all motors will receive the command, but no response
    Data Length: the length of the message which is about to send 
                (length+3=total length, where 3 is the header and checksum)
    Command: All type of commands which is already been defined
    Paramerters: all information for the type of command required
    Check Sum: checksum = (ID + Length + cmd + prm1 + ... + prmN)
    '''

    def calculate_checksum(self, id, length, cmd, *params):
        """Calculate the check sum
        Args:
            id (int): 消息 id。
            length (int): 数据长度。
            cmd (int): 命令字。
            *params: 可变参数，表示参数列表。
        Returns:
            int: 计算出的校验和。
        """
        total = id + length + cmd + sum(params)
        if total > 255:
            total = total % 256
        checksum = ~total & 0xFF  # 保留最低字节
        print("checksum: ", checksum)
        return checksum

    def construct_command_message(self, id, cmd, params):
        """构造命令消息。

        Args:
            motor_id (int): 舵机 ID (0-253)。
            cmd (int): 命令字。
            params (list): 参数列表。

        Returns:
            bytearray: 构造的命令消息。
        """
        length = 3 + len(params)  # 参数数量
        checksum = self.calculate_checksum(id, length, cmd, *params)

        # construct the message
        packet = [0x55, 0x55, id, length, cmd]
        for param in params:
            packet.append(param)
        packet.append(checksum)
        # Print the packet in hexadecimal format
        hex_packet = [hex(x) for x in packet]
        print("packet (hex): ", hex_packet)
        return packet

    def send_command(self, packet):
        with self.serial_mutex:
            self.__write_serial(packet)
            
    def set_servo_position(self, id, position, duration=None):
        '''
        驱动串口舵机转到指定位置(drive the serial servo to rotate to the designated position)
        :param id: 要驱动的舵机id(param id: the  servo id to be driven)
        :pulse: 位置(pulse: position)
        :use_time: 转动需要的时间(use_time: time taken for rotation)
        '''
        # print("id:{}, pos:{}, duration:{}".format(hand_id, position, duration))
        if duration is None:
            duration = 20
        duration = 0 if duration < 0 else 30000 if duration > 30000 else duration
        position = 0 if position < 0 else 1000 if position > 1000 else position
        duration = int(duration)
        position = int(position)
        pos_low = int(position & 0xFF)
        pos_high = int(position >> 8)
        duration_low = int(duration & 0xFF)
        duration_high = int(duration >> 8)
        param = [pos_low, pos_high, duration_low, duration_high]
        packet = self.construct_command_message(id, self.CMD_SERVO_MOVE_TIME_WRITE, param)

        

# Example usage:
if __name__ == "__main__":
    commander = MotorCommander()
    commander.calculate_checksum(0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03)
    commander.set_servo_position(1, 500, 1000)