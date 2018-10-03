from bluetooth import *


class SimpleBluetoothServer:

    def __init__(self):

        self.server_sock = BluetoothSocket(RFCOMM)
        self.server_sock.bind(("", PORT_ANY))
        self.server_sock.listen(1)

        self.port = self.server_sock.getsockname()[1]

        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

        advertise_service(self.server_sock, "DummySurveillanceServer",
                          service_id=uuid,
                          service_classes=[uuid, SERIAL_PORT_CLASS],
                          profiles=[SERIAL_PORT_PROFILE],
                          # protocols = [ OBEX_UUID ]
                          )

    def start(self):

        print("\nRFCOMM Server started at channel %d" % self.port)

        while True:

            client_sock, client_info = self.server_sock.accept()
            print("Accepted connection from ", client_info)

            try:
                while True:
                    data = client_sock.recv(1024)
                    if len(data) == 0:
                        print("Empty packets received.")
                        break
                    print("received [%s]" % data)
            except IOError:
                pass

            print("Client disconnected.")

            client_sock.close()
            #self.server_sock.close()
            print("Reopening server.")
