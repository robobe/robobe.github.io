from ctypes import Structure, byref, c_bool, c_int, c_float, c_char, sizeof
from ctypes import memmove

# https://itecnote.com/tecnote/python-how-to-copy-bytes-from-a-ctypes-structure-to-a-buffer-created-from-create_string_buffer/

class MyData2(Structure):
    _fields_: [('m_bool', c_bool),
        ('m_int', c_int),
        ('m_float', c_float)]

data = MyData2()
data.m_bool = True
data.m_int = 1
data.m_float = 2.0
buf = (c_char*100)()
memmove(buf, byref(data), sizeof(data))