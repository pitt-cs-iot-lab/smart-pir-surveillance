import socket
import sys
from contextlib import closing



def main():

    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:

        print('Starting Server on {}'.format(HOST))
        print(sys.version_info)

        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()

        with conn:
            print('Connected by', addr)
            while True:
                data = conn.recv(1024)
                print(data)
                if not data:
                    print('All data received!')
                    conn.
                    #break
                conn.sendall(data)


if __name__ == '__main__':
    main()
