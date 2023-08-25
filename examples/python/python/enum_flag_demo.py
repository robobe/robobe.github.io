# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
from enum import Flag, auto, IntFlag


class Condition(Flag):
    SUNNY = auto()
    WINDY = auto()
    RAIN = auto()
    CLOUD = auto()
    PARTIAL_CLOUD = auto()

current_condition: Condition = Condition.SUNNY | Condition.WINDY | Condition.PARTIAL_CLOUD
print(dir(current_condition))

print(type(current_condition))
print(Condition.WINDY in current_condition)
print(Condition.CLOUD in current_condition)
print(current_condition & Condition.SUNNY)
print(current_condition & Condition.CLOUD)

new_condition = Condition(7)
print(new_condition)

print(1 == Condition.SUNNY)

class Condition2(IntFlag):
    SUNNY = auto()
    WINDY = auto()
    RAIN = auto()

print(1 == Condition2.SUNNY)
print(Condition2.WINDY+1)

from typing import Literal


Status = Literal[404, 200, 500]

def set_x(value: Status):
    print(value)

set_x(404)
set_x(1404)



