#!/usr/bin/python3

import socket
import serial
SERVER = "127.0.0.1"
PORT = 50000

coordinator = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
coordinator.connect((SERVER, PORT))


def get_self_dist():
    with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as opened_serial:
        opened_serial.write('dis'.encode())
        opened_serial.flush()
        while True:
            data_from_server = opened_serial.readline()[:-2]
            if not data_from_server == '':
                return data_from_server.decode()


def move(dist):
    with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as opened_serial:
        opened_serial.write(str(dist).encode())
        opened_serial.flush()
        while True:
            data_from_server = opened_serial.readline()[:-2].decode()
            if not data_from_server == '':
                print(data_from_server)
            if data_from_server == 'Done':
                break


def is_number(var):
    try:
        if var == int(var):
            return True
    except Exception:
        return False


while True:
    in_data = coordinator.recv(1024).decode()
    if not in_data == '':
        if in_data == 'cgd':
            dd = get_self_dist()
            print('dist', dd)
            coordinator.send(dd.encode())
        else:
            print('move', in_data)
            move(in_data)
            coordinator.send('done'.encode())
coordinator.close()
