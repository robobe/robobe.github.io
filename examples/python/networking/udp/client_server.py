import socket
import logging
import time 
import multiprocessing
import struct
from typing import NamedTuple
# https://www.pythonpool.com/struct-pack/
# https://zetcode.com/python/namedtuple/

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
log = logging.getLogger(__name__)

class MyData(NamedTuple):
    id: int
    age: bytes
    name: str



def server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('', 12000))

    while True:
        message, address = server_socket.recvfrom(1024)
        data = struct.unpack("IB4s", message)
        data = MyData._make(data)
        log.debug(data.name)
        server_socket.sendto(b"ack", address)


def client():
    for pings in range(10):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.settimeout(1.0)
        data = MyData(pings, pings, b"test")
        data = struct.pack('IB4s', *data)
        
        message = data
        addr = ("127.0.0.1", 12000)

        start = time.time()
        client_socket.sendto(message, addr)
        try:
            data, server = client_socket.recvfrom(1024)
            end = time.time()
            elapsed = end - start
            log.debug(f'{data} {pings} {elapsed}')
        except socket.timeout:
            log.error('REQUEST TIMED OUT')
    

    
if __name__ == "__main__":
    p_server = multiprocessing.Process(target=server)
    p_client = multiprocessing.Process(target=client)
    p_server.start()
    p_client.start()
    p_server.join()
    p_client.join()
