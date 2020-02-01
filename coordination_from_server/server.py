#!/usr/bin/python3

import socket
import threading

import serial


def get_self_dist():
    with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as opened_serial:
        opened_serial.write('dis'.encode())
        opened_serial.flush()

        while True:
            data_from_machine = opened_serial.readline()[:-2]

            if not data_from_machine == '':
                return data_from_machine.decode()


def get_distance(clientsocket):
    clientsocket.send('cgd'.encode())

    while True:
        msg = clientsocket.recv(1024).decode()

        if not msg == '':
            print(addr, ' >> ', msg)
            dist = int(msg)

            return dist - self_dist


def coordinate():
    while True:
        for addr, clientsocket in client_data:
            if addr not in client_done:
                dd = get_distance(clientsocket)

                while abs(dd) > ERROR:
                    clientsocket.send(str(dd).encode())
                    clientsocket.recv(1024).decode()
                    dd = get_distance(clientsocket)
                client_done.append(addr)


s = socket.socket()
host = '127.0.0.1'  # socket.gethostname() # Get local machine name
port = 50000
client_data = []
client_done = []
self_dist = int(get_self_dist())

ERROR = 2
print("Line: ", self_dist)
print('Server started!')
print('Waiting for clients...')

s.bind((host, port))  # Bind to the port
s.listen(5)  # Now wait for client connection.

threading.Thread(target=coordinate, args=()).start()

while True:
    c, addr = s.accept()
    client_data.append((addr, c))
    print('Got connection from', addr)
s.close()
