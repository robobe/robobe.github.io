# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=disallowed-name

from typing import Generic, TypeVar

T = TypeVar("T")
U = TypeVar("U")


class MyClass(Generic[T, U]):
    def __init__(self, foo: T, bar: U) -> None:
        self.foo = foo
        self.bar = bar

    def get_foo(self) -> T:
        return self.foo

    def get_bar(self) -> U:
        return self.bar

if __name__ == "__main__":
    obj = MyClass[str, int]("name", 10)
    print(obj.get_foo())