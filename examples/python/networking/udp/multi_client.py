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
    # name: str

def client():
    for index in range(10):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data = struct.pack('I', index)
        
        message = data
        addr = ("127.0.0.1", 1234)

        start = time.time()
        client_socket.sendto(message, addr)
        try:
            data, server = client_socket.recvfrom(1024)
            end = time.time()
            elapsed = end - start
            log.debug(f'{data} {index} {elapsed}')
        except socket.timeout:
            log.error('REQUEST TIMED OUT')
    

    
if __name__ == "__main__":
    client1 = multiprocessing.Process(target=client)
    client1.start()
    client2 = multiprocessing.Process(target=client)
    client2.start()
    client1.join()
    client2.join()
