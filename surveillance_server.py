from simple_bluetooth_server import *
from simple_tcp_server import *
from threading import Thread


def main():

    b_server = SimpleBluetoothServer()

    b_thread = Thread(target=b_server.start)
    b_thread.start()

    tcp_server = SimpleTCPServer()
    tcp_thread = Thread(target=tcp_server.start)
    tcp_thread.start()

    #print('All servers have been started.')


if __name__ == '__main__':
    main()
