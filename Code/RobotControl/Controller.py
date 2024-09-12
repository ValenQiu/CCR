# -*- coding: utf-8 -*-
import sys
import serial
import serial.tools.list_ports
from threading import Lock
import time
from pynput import keyboard

ports_list = list(serial.tools.list_ports.comports())  # 获取所有串口设备实例
if len(ports_list) <= 0:
    print("No serial devices avaliable!")
else:
    print("avaliable serial devices:")
    for port in ports_list:  # 依次输出每个设备对应的串口号和描述信息
        print(list(port)[0], list(port)[1])  # COM4 USB-SERIAL CH340 (COM4)


class Controller():
    
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
    CMD_BATTERY_VOLTAGE = 15
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

    def __init__(self, port):
        """open the port, initialization (open the serial port. Initialize the parameter)"""
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.timeout = 10
            self.ser = serial.Serial(port, 9600, timeout=0.5)
            if self.ser.isOpen():  # 判断串口是否成功打开
                print("port open successfully")
                print(self.ser.name)  # 输出串口号，即COM4
            else:
                print("port open fail")
            self.port_name = port
        except:
            print("error")

    def __del__(self):
        self.close()

    def close(self):
        """
               close the serial port.
               """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()
        if self.ser.isOpen():  # 判断串口是否关闭
            print("port not close yet")
        else:
            print("port is closed")

    def __write_serial(self, data):
        self.ser.flushInput()
        self.ser.write(data)
        # print("data", data)
        time.sleep(0.0084)

    def __read_response(self, length):
        data = []
        try:
            data.extend(self.ser.read(length))
            if not data[0:2] == [0x55, 0x55]:
                raise Exception('Wrong packet prefix' + str(data[0:2]))
            # data.extend(self.ser.read(data[3] - 1))
        except Exception as e:
            raise DroppedPacketError('Invalid response received from hand ' + str(hand_id) + ' ' + str(e))

        return data
    '''
    Communication protocol of the motor control board:
    -----------------------------------------------------
    |   Header  | Data Length | Command |  Parameters   |
    |---------------------------------------------------|
    | 0x55 0x55 |   length    |   cmd   | prm1 ... prmN |
    -----------------------------------------------------
    where:
    Header: two 0x55, means the message is sended
    Data Length: the length of the message which is about to send 
                (length+3=total length, where 3 is the header and checksum)
    Command: All type of commands which is already been defined
    Paramerters: all information for the type of command required
    '''
    
    ### Basic functions
    def convert_byte_to_high_low_values(self, x):
        '''This function is used to calculate the high byte and low byte'''
        x = int(x)
        low_byte = int(x & 0xFF)
        high_byte = int(x >> 8)
        # print("low_byte: ",low_byte)
        # print("high_byte: ",high_byte)
        return low_byte, high_byte
    
    def convert_higt_low_values_to_byte(self, low_byte, high_byte):
        x = low_byte + (high_byte << 8)
        return x
    
    def get_response(self, packet, respon_length):
        data = []
        with self.serial_mutex:
            for i in range(10):
                try:
                    self.__write_serial(packet)
                    # wait for response packet from the motor
                    # read response
                    data = self.__read_response(respon_length)
                    # timestamp = time.time()
                    # data.append(timestamp)
                    break
                except Exception as e:
                    if i == 49:
                        raise e
        return data

    def read(self):
        # Number of bytes following standard header (0xFF, 0xFF, id, length)
        length = 5  # instruction, address, size, checksum
    
        ##计算校验和
        checksum = 255 - ((id + length + cmd) % 256)
        # packet: 0x55  0x55  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0x55, 0x55, id, length, cmd, checksum]
    
        data = []
        with self.serial_mutex:
            for i in range(10):
                try:
                    self.__write_serial(packet)
                    # wait for response packet from the motor
                    # read response
                    data = self.__read_response(length)
                    timestamp = time.time()
                    data.append(timestamp)
                    break
                except Exception as e:
                    if i == 49:
                        raise e
        return data

    def write(self, cmd, params):
        # data received from "set_servo_position"
        # hand_id, CMD_SERVO_MOVE, (loVal, hiVal, loTime, hiTime)
        """ Write the values from the "data" list to the servo with "id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.
        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """
        # Number of bytes following standard header (0xFF, 0xFF, id)
        length = 2 + len(params)  # length, cmd, params, checksum
        # print("length: ", length)
        # Check Sum = ~ ((ID + LENGTH + COMMAND + PARAM_1 + ... + PARAM_N) & 0xFF)
        # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0x55, 0x55, length, cmd]
        for param in params:
            packet.append(param)
        print("packet", packet)
        with self.serial_mutex:
            self.__write_serial(packet)

    def get_servo_position(self, id):
        print("program is here")
        response = self.read_servo_position(id, self.CMD_SERVO_POS_READ)
        print("response",response)
        if response:
            self.exception_on_error(response[4], id, 'fetching present position')
            return response[6] + (response[7] << 8)

    def read_servo_position(self, id, cmd):
        # Number of bytes following standard header (0xFF, 0xFF, length, cmd, number_of_servo, servo_id)
        cmd = self.CMD_SERVO_POS_READ
        length = 4  # instruction, address, size, checksum

        # packet: 0x55  0x55  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
        packet = [0x55, 0x55, length, cmd, 1, id]

        data = []
        with self.serial_mutex:
            for i in range(10):
                try:
                    self.__write_serial(packet)
                    # wait for response packet from the motor
                    # read response
                    data = self.__read_response(length+3)
                    timestamp = time.time()
                    data.append(timestamp)
                    break
                except Exception as e:
                    if i == 49:
                        raise e
        return data

    def set_servo_position(self, id, position, duration=None):
        '''
        驱动串口舵机转到指定位置(drive the serial servo to rotate to the designated position)
        :param id: 要驱动的舵机id(param id: the  servo id to be driven)
        :pulse: 位置(pulse: position)
        :use_time: 转动需要的时间(use_time: time taken for rotation)
        '''
        if duration is None:
            duration = 20
        duration = 0 if duration < 0 else 30000 if duration > 30000 else duration
        position = 0 if position < 0 else 1000 if position > 1000 else position
        loVal, hiVal = self.convert_byte_to_high_low_values(position)
        loTime, hiTime = self.convert_byte_to_high_low_values(duration)
        params = [1, loTime, hiTime, id, loVal, hiVal]
        self.write(self.CMD_SERVO_MOVE, params)

    def stop(self, id):
        '''
        停止舵机运行(stop servo rotation)
        :param id:
        :return:
        '''
        self.write(id, self.CMD_SERVO_MOVE_STOP, ())
        
    def get_battery_voltage(self):
        '''
        send command:
        -----------------------------------------------------
        |   Header  | Data Length | Command |  Parameters   |
        |---------------------------------------------------|
        | 0x55 0x55 |      2      |   15    |      NaN      |
        -----------------------------------------------------
        
        Return:
        -----------------------------------------------------
        |   Header  | Data Length | Command |  Parameters   |
        |---------------------------------------------------|
        | 0x55 0x55 |      4      |   15    |  Prm1, Prm2   |
        -----------------------------------------------------
        Prm1: Low byte of the voltage
        Prm2: High byte of the voltage
        '''
        packet = [0x55, 0x55, 2, 15]
        self.__write_serial(packet)
        # wait for response packet from the motor
        data = []
        with self.serial_mutex:
            for i in range(10):
                try:
                    self.__write_serial(packet)
                    # wait for response packet from the motor
                    # read response
                    data = self.__read_response(6)
                    break
                except Exception as e:
                    if i == 49:
                        raise e
        # print(data)
        voltage_low, voltage_high = data[4], data[5]
        voltage = self.convert_higt_low_values_to_byte(voltage_low, voltage_high)
        print("Battery Voltage: ", voltage, "mV")
        return voltage
    
    def servo_unload(self, ids):
        '''
        ------------------------------------------------------------
        |   Header  |    Data Length   | Command |   Parameters    |
        |----------------------------------------------------------|
        | 0x55 0x55 | num of servo + 3 |   20    | Par1, ..., ParN |
        ------------------------------------------------------------
        Par1: Number of servos to unload
        Par2: Id of servo A
        Par3: Id of servo B
        Par...: Id of servo X
        '''
        ''' The input ids should be an array or an integer'''
        if ids is None:
            print("No servo ID input!")
            return
        # Check if ids is a single integer
        if isinstance(ids, int):
            packet = [0x55, 0x55, 0x04, 0x14, 0x01, ids]
            print(packet)
        ''' TO BE FINISH '''
        self.__write_serial(packet)
        
    def read_multi_servo_positions(self, ids):
        '''
        Sending:
        ------------------------------------------------------------
        |   Header  |    Data Length   | Command |   Parameters    |
        |----------------------------------------------------------|
        | 0x55 0x55 | num of servo + 3 |   21    | Par1, ..., ParN |
        ------------------------------------------------------------
        Par1: Number of servos to read
        Par2: Id of servo A
        Par3: Id of servo B
        Par...: Id of servo X
        
        Respinsing:
        ------------------------------------------------------------
        |   Header  |    Data Length   | Command |   Parameters    |
        |----------------------------------------------------------|
        | 0x55 0x55 | num of servo + 3 |   21    | Par1, ..., ParN |
        ------------------------------------------------------------
        Par1: Number of servos to read
        Par2: Id of servo A
        Par3: Low byte of servo A's position
        Par4: High byte of servo A's position
        Par...: Id of servo X
        '''
        # Constructing the command message
        if ids is None or ids == []:
            print("The input cannot be none or empty")
            return
        servo_num = len(ids)
        length = servo_num + 3
        cmd = 21
        packet = [0x55, 0x55, length, cmd, servo_num]
        params = ids
        for param in params:
            packet.append(param)
        print("packet", packet)
        respon_length = servo_num * 3 + 5
        data = self.get_response(packet, respon_length)
        # print(data)
        servo_positions = data[5:]
        # print(servo_positions)
        ids = []
        positions = []
        for i in range(0, servo_num):
            id = servo_positions[i*3]
            position_low = servo_positions[i*3 + 1]
            position_high = servo_positions[i*3 + 2]
            position = self.convert_higt_low_values_to_byte(position_low, position_high)
            ids.append(id)
            positions.append(position)
        print("IDs: ", ids)
        print("Positions: ", positions)
        return ids, positions
    
    def set_and_write_servo_position(self, id, position, duration=None):
        """
        Drive the serial servo to rotate to the designated position and write the values
        to the servo with the specified id.

        :param id: The servo ID to be driven.
        :param position: The target position (0-1000).
        :param duration: The time taken for rotation (0-30000).
        """
        # Handle duration
        if duration is None:
            duration = 20
        duration = 0 if duration < 0 else 30000 if duration > 30000 else duration

        # Handle position
        position = 0 if position < 0 else 1000 if position > 1000 else position
        print("position: ", position)

        # Convert position and duration to low and high bytes
        loVal, hiVal = self.convert_byte_to_high_low_values(position)
        loTime, hiTime = self.convert_byte_to_high_low_values(duration)

        # Prepare the parameters for the write command
        params = [1, loTime, hiTime, id, loVal, hiVal]

        # Calculate packet length
        length = 2 + len(params)  # length, cmd, params, checksum

        # Create the packet with the standard header
        packet = [0x55, 0x55, length, self.CMD_SERVO_MOVE]
        for param in params:
            packet.append(param)

        print("packet: ", [hex(byte) for byte in packet])

        with self.serial_mutex:
            self.__write_serial(packet)
    
    def set_multi_servo_positions(self, ids, positions, time):
        '''
        Sending:
        ----------------------------------------------------------------
        |   Header  |       Data Length    | Command |   Parameters    |
        |--------------------------------------------------------------|
        | 0x55 0x55 | num of servo * 3 + 5 |   3    | Par1, ..., ParN  |
        ----------------------------------------------------------------
        Par1: Number of servos to control
        Par2: Low byte of time
        Par3: High byte of time
        Par4: ID of the servo motor
        Par5: Low byte of servo A's position
        Par6: High byte of servo A's position
        Par...: Other servo motors
        '''
        
        # Validate inputs
        if ids is None or not ids:
            print("The input cannot be none or empty")
            return

        if len(ids) != len(positions):
            print("The lengths of ids and positions must match.")
            return

        servo_num = len(ids)
        length = servo_num * 3 + 5  # Ensure this calculation is correct
        cmd = self.CMD_SERVO_MOVE

        # Convert time to low and high bytes
        time_low, time_high = self.convert_byte_to_high_low_values(time)

        # Construct the packet
        packet = [0x55, 0x55, length, cmd, servo_num, time_low, time_high]
        
        for i in range(servo_num):
            id = ids[i]
            position = positions[i]
            position_low, position_high = self.convert_byte_to_high_low_values(position)
            packet.extend([id, position_low, position_high])
        
        # Send the packet
        with self.serial_mutex:  # Ensure thread safety
            self.__write_serial(packet)
            print("Packet to send:", packet)
        
    def exception_on_error(self, error_code, hand_id, command_failed):
        global exception
        exception = None

        if not isinstance(error_code, int):
            ex_message = '[servo #%d on %s@%sbps]: %s failed' % (
            hand_id, self.ser.port, self.ser.baudrate, command_failed)
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return


class ChecksumError(Exception):
    def __init__(self, hand_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from motor %d does not match the expected one (%d != %d)' \
                       % (hand_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum

    def __str__(self):
        return self.message


class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message


class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message

    def __str__(self):
        return self.message



# controller = Controller('COM3')
# time.sleep(1)

## Examples

# test_value = 500
# low_value, high_value = controller.convert_byte_to_high_low_values(test_value)
# # print("result: ", hex(high_value), hex(low_value))
# return_value = controller.convert_higt_low_values_to_byte(low_value, high_value)
# print(return_value)

# controller.set_servo_position(11, 1000, 500)
# controller.get_battery_voltage()
# position = controller.get_servo_position(11)
# print(position)
# controller.servo_unload(22)
# controller.read_multi_servo_positions([11, 12, 21, 22])

# ids = [11, 12, 21, 22]
# positions = [1000, 1000, 1000, 1000]
# ids = [11, 12, 21, 22]
# positions = [500, 500, 500, 500]
# # positions = [1000, 1000, 1000, 1000]
# controller.set_multi_servo_positions(ids, positions, 500)

# packet = [0x55, 0x55, 0x0B, 0x03, 0x02, 0x20, 0x03, 0x02, 0x20, 0x03, 0x09, 0x20, 0x03]
# controller.__write_serial(packet)

# controller.set_and_write_servo_position(11, 500, 500)
# params = []

# command = [85, 85, 11, 3, 2, 244, 1, 11, 232, 3, 12, 232, 3]
# controller.ser.write(command)
# time.sleep(0.31)