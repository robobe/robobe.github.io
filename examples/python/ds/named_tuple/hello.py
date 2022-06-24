from typing import NamedTuple

class MyTuple(NamedTuple):
    id: int
    name: str

t1 = MyTuple(1, "A")
t2 = MyTuple(*(2, "B"))
t3 = MyTuple(**{
    "id": 3,
    "name": "C"})
t4 = MyTuple._make([4, "D"])

print(t1)
print(t2)
print(t3)
print(t4)