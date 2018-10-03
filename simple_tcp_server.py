import socket
import sys
from contextlib import closing


class SimpleTCPServer:

    def __init__(self, ip='192.168.1.121', port=65432):

        self.server_ip = ip
        self.server_port = port

    def start(self):

        with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:

            s.bind((self.server_ip, self.server_port))
            s.listen(1)

            requests = 0

            while True:

                print('\nStarting TCP Server on {}'.format(self.server_ip))
                #print(sys.version_info)
                print('Waiting for request nr: {}'.format(requests))

                conn, addr = s.accept()

                with conn:
                    print('Connected by', addr)
                    while True:
                        data = conn.recv(1024)
                        print(data)
                        if not data:
                            print('All data received!')
                            break
                        conn.sendall(data)

                requests += 1
