#!/usr/bin/python

__author__ = 'gdiaz'

class fileManager(object):
    def __init__(self, file_name, debug = False):
        self.debug = debug
        self.file_path = "/home/bender/uchile_ws/ros/bender/base_ws/src/uchile_tools/uchile_battery_sensor/src/uchile_battery_sensor/data"
        self.file_name = file_name
        self.file_ext = ".txt"
        self.full_name = self.file_path+self.file_name+self.file_ext
        self.data_file = open(self.full_name, "ab+")

    def stop(self):
        self.data_file.close()

    def DEBUG_PRINT(self, msg_type, msg):
        if not(self.debug):
            return
        if msg_type == "info":
            print chr(27)+"[0;32m"+"[INFO]: "+chr(27)+"[0m" + msg
        elif msg_type == "warn":
            print chr(27)+"[0;33m"+"[WARN]: "+chr(27)+"[0m" + msg
        elif msg_type == "error":
            print chr(27)+"[0;31m"+"[ERROR]: "+chr(27)+"[0m" + msg
        elif msg_type == "alert":
            print chr(27)+"[0;34m"+"[ALERT]: "+chr(27)+"[0m" + msg
        else:
            print "NON implemented Debug print type"

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1):
            sum += packet[j]
        return sum

    def bytesEncode(self, number):
        int_part = int(number)                  #[0-65535]
        decimal_part = (number - int_part)*100  #[0-99]

        NH = (abs(int_part))>>8                 #Number High Byte
        NL = (abs(int_part)) & 0x00ff           #Number Low Byte
        D = abs(int(decimal_part))              #Decimal part (7 bits)
        SD = D                                  #Sign and Decimal Byte
        if (number<=0):
            SD = D|0b10000000                   #Sign bit

        return [NH, NL, SD]

    def encode(self, data):
        packet = [0,0,0,0,0,0,0,0,0,0,0,0,0]

        num1_bytes = self.bytesEncode(data[0])
        num2_bytes = self.bytesEncode(data[1])
        num3_bytes = self.bytesEncode(data[2])
        num4_bytes = self.bytesEncode(data[3])

        packet[0] = num1_bytes[0]
        packet[1] = num1_bytes[1]
        packet[2] = num1_bytes[2]

        packet[3] = num2_bytes[0]
        packet[4] = num2_bytes[1]
        packet[5] = num2_bytes[2]

        packet[6] = num3_bytes[0]
        packet[7] = num3_bytes[1]
        packet[8] = num3_bytes[2]

        packet[9] = num4_bytes[0]
        packet[10] = num4_bytes[1]
        packet[11] = num4_bytes[2]

        packet[12] = self.checksum(packet, 13) & 0x00ff

        return packet

    def getNumberElements(self, byte1, byte2, byte3):
        int_part = (byte1<<8)|byte2
        sign = byte3>>7
        decimal_part = byte3&(0b01111111)
        return [int_part, sign, decimal_part]

    def getNumber(self, number_elements):
        int_part = number_elements[0]
        sign = number_elements[1]
        decimal_part = number_elements[2]
        if (sign == 0):
            number = int_part+(float(decimal_part)/100)
        else:
            number = -(int_part+(float(decimal_part)/100))
        return number

    def saturate(self, data, MIN, MAX, nm, i):
        return data
        if data>MAX:
            self.DEBUG_PRINT("error", "Data over = "+str(data))
            self.DEBUG_PRINT("error", "nm"+str(i)+ "= "+str(nm))
            # return MAX
        elif data<MIN:
            self.DEBUG_PRINT("error", "Data under = "+str(data))
            self.DEBUG_PRINT("error", "nm"+str(i)+ "= "+str(nm))
            # return MIN
        # else:
        # return data

    def decode(self, packet):
        if packet[1]>=11 or packet[4]>=11 or packet[7]>=11 or packet[10]>=11:
            self.DEBUG_PRINT("warn", "issue frame = "+str(packet))

        frame_id = packet[0]

        number1_elements = self.getNumberElements(packet[1], packet[2], packet[3])
        data1 = self.saturate(self.getNumber(number1_elements), 0, 3000, number1_elements, 1)

        number2_elements = self.getNumberElements(packet[4], packet[5], packet[6])
        data2 = self.saturate(self.getNumber(number2_elements), 0, 3000, number2_elements, 2)

        number3_elements = self.getNumberElements(packet[7], packet[8], packet[9])
        data3 = self.saturate(self.getNumber(number3_elements), 0, 3000, number3_elements, 3)

        number4_elements = self.getNumberElements(packet[10], packet[11], packet[12])
        data4 = self.saturate(self.getNumber(number4_elements), 0, 3000, number4_elements, 4)

        return [data1/360, data2, data3, data4]

    def to_file(self, data, sz):
        self.data_file = open(self.full_name, "ab+")
        for i in range(0,sz):
            self.data_file.write(str(data[i]))
            self.data_file.write(";")
        self.data_file.write("\n")
        self.data_file.close()

    def save_data(self, packet, instant_energy):
        data = self.decode(packet)
        data2=data.append(instant_energy)
        self.DEBUG_PRINT("info", "Data = "+str(data))
        self.to_file(data, 5)


if __name__ == '__main__':
    file_name = raw_input("file_name: ")
    file_manager = fileManager(file_name, debug = True)
    # test
    n_test = 10
    packet_test = [1,6,1,8,16]
    print "packet_encode = " + str(file_manager.encode([-102.92, -121.14, -51.34, -67.15]))
    print "data_decode = "+str(file_manager.decode([0, 102, 220, 0, 121, 142, 0, 51, 162, 0, 67, 143, 240]))
    # data = [123.234, -123.435]
    # packet_test2 = file_manager.encode(data)
    # print "starting test ..."
    # print "data = "+str(data)
    # print "packet_encode = " + str(packet_test2)
    # print "data_decode = "+str(file_manager.decode(packet_test2))
    # i = 0
    # while i<n_test:
    #     # file_manager.to_file(packet_test, 5)
    #     file_manager.save_data(packet_test2, 5)
    #     i+=1
    print "test finish ..."
    file_manager.stop()