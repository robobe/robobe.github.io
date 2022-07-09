---
title: Bytearray
tags:
    - python
---

The bytearray class is a **mutable** sequence of integers in the range 0 <= x < 256.

```python
data = bytearray(10)
data
bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
```
!!! note "mutable vs immutable"
    A mutable object can be changed after it's created, and an immutable object can't. 

## bytearray, bytes and string
`bytes` and `bytearray` classes both hold arrays of bytes, where each byte can take on a value between 0 and 255. The primary difference is that a **bytes object is immutable**, meaning that once created, you cannot modify its elements. By contrast, a bytearray object allows you to modify its elements
     
```python
# create array with 10 bytes
data = bytearray(10)
b = bytes(data)
>>> b
b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
```

```python
my_str = "abcd"
>>> my_data = bytearray(my_str, 'ascii')
>>> my_data
bytearray(b'abcd')
>>> len(my_data)
4

>>> my_data = bytearray(my_str, 'utf-16')
>>> len(my_data)
10
```

---

## more
```python
>>> data=bytearray([65,66,67])
>>> data
bytearray(b'ABC')

# show etch value as hex
>>> [hex(c) for c in data]
['0x41', '0x42', '0x43']

# append
>>> data.append(68)
data
bytearray(b'ABCD')

# replace
>>> data[1:3] = [0x62, 0x63]
>>> data
bytearray(b'AbcD')

# extend
>>> data.extend([0x45, 0x46])
>>> data
bytearray(b'AbcDEF')

# extend
>>> new_data = bytearray(b"GH")
>>> new_data
bytearray(b'GH')
>>> data + new_data
bytearray(b'AbcDEFGH')
```

     

