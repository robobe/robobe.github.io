# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
from typing import TypeVar
from typing_extensions import Protocol

T = TypeVar("T")


class SupportAdd(Protocol[T]):
    def add(self, a: T, B: T) -> T:
        pass


class SimpleAdd:
    def add(self, a: int, b: int) -> int:
        return a + b


class SimpleAdd2:
    def add(self, a: str, b: str) -> str:
        return a + b


simple_add: SupportAdd[int] = SimpleAdd()
simple_add2: SupportAdd[str] = SimpleAdd2()
