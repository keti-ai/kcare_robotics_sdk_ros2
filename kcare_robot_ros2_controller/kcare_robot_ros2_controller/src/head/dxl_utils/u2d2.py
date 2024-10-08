import os
from dynamixel_sdk import *

# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

class U2D2:
    def __init__(self, logger):
        self.logger = logger
        
        self.port = None
        self.isOpened = False

        self.buadrate = None
        self.protocol = None

        self.port_handler = None
        self.packet_handler = None

    def set(self, baudrate, protocol, device_name):
        self.baudrate = baudrate
        self.protocol = protocol
        self.port = device_name

    def init(self):
        port_stat = False
        baud_stat = False

        try:
            self.close()
        except:
            pass

        if self.port is None:
            try_cnt = 1
            while True:
                
                if self.port is not None:
                    break

                else:
                    self.logger.warning(f"U2D2 port not found. Try: {try_cnt}.")
                    try_cnt += 1

                if try_cnt >= 5:
                    self.logger.info(f"U2D2 port not found. Check the U2D2 connection.")
                    break

                time.sleep(0.05)

            if self.port is not None:
                self.init()
        else:
            try:
                self.port_handler = PortHandler(self.port)
                self.packet_handler = PacketHandler(self.protocol)

            except Exception as e:
                self.logger.error(f"Failed connect U2D2. {e}")

            try:
                if self.port_handler.openPort():
                    port_stat = True
                    self.logger.info(f"U2D2 connection successful. Port: {self.port}.")

                else:
                    self.logger.error(f"Failed open U2D2.")
                    getch()
                    quit()

            except Exception as e:
                self.logger.error(f"Failed open U2D2. {e}")

            try:
                if self.port_handler.setBaudRate(self.baudrate):
                    baud_stat = True
                    self.logger.info(f"U2D2 baudrate setting success. Baudrate: {self.baudrate}.")
                else:
                    self.logger.error("Failed to change the baudrate.")
                    getch()
                    quit()

            except Exception as e:
                self.logger.error(f"Failed to change the baudrate. {e}")

        if port_stat and baud_stat:
            self.isOpened = True

    def close(self):
        if self.port_handler is not None:
            try:
                self.port_handler.closePort()
            except:
                pass

        self.port_handler = None
        self.packet_handler = None

        self.isOpened = False