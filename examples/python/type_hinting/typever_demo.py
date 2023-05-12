import decimal
from typing import TypeVar

Number = TypeVar("Number", int, float, decimal.Decimal)
T = TypeVar("T")

def foo(arg: T) -> T:
    return arg

def double(value: Number) -> Number:
    return value * 2

result: int
result = double(5)
