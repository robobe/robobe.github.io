import socket
import select

sockets = []

for port in range(1234,1236):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', port))
    sockets.append(server_socket)

empty = []
while True:
    readable, writable, exceptional = select.select(sockets, empty, empty)
    for s in readable:
        (client_data, client_address) = s.recvfrom(1024)
        print(client_address, client_data)
        s.sendto(client_data, client_address)
for s in sockets:
   s.close()