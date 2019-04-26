import pyautogui
import serial
import argparse
import time
import logging


class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'L', 'skip': 's', 'pause': 'space', 'rewind': 'left',
                       'fast-foward': 'right', 'fullscreen': 'f', 'mute': 'm'}  # Fast forward (10 seg) pro Youtube


class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  # remove delay

    def update(self):
        # Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            # logging.debug("Received INCOMING: {}".format(self.incoming))

        data = self.ser.read(8)
        # logging.debug("Received DATA: {}".format(data))
        data_str = bytearray(data).decode()
        pause_play, rrewind, ff, skip, fullscreen, mute, nextep, Xis = data_str
        # logging.debug(f"data_str:{data_str}")
        # print(data_str)
        if pause_play == '1':
            print("pause")
            logging.debug("Sending pause/play")
            pyautogui.keyDown(self.mapping.button['pause'])
        elif rrewind == '1':
            logging.debug("Sending rewind")
            pyautogui.keyDown(self.mapping.button['rewind'])
        elif ff == '1':
            logging.debug("Sending fastforward")
            pyautogui.keyDown(self.mapping.button['fast-foward'])
        elif skip == '1':
            logging.debug("Sending skip")
            pyautogui.keyDown(self.mapping.button['skip'])
        elif fullscreen == '1':
            logging.debug("Sending fullscreen")
            pyautogui.keyDown(self.mapping.button['fullscreen'])
        elif mute == '1':
            logging.debug("Sending mute")
            pyautogui.keyDown(self.mapping.button['mute'])
        # if nextep == b'1':
        #     logging.info("Sending press")
        #     self.j.set_button(self.mapping.button['pause'])

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        # pyautogui.press(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface',
                          type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(
        args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(
            port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
