# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
# from typing import TypeVar, Generic, List

# T = TypeVar("T", str, bytes)

# def func(arg: T) -> T:
#     return arg

# func_with_string = func("a")
# func_with_byte = func(b"a")


# from typing import TypeVar, Generic




# class GenericClass(Generic[T]):
#     def __init__(self, value: T) -> None:
#         self.value = value

#     def get_value(self) -> T:
#         return self.value

#     def set_value(self, value: T) -> None:
#         self.value = value


# obj1 = GenericClass[int](1)
# obj2 = GenericClass[str]("hello")

# obj1.set_value("a")


from typing import TypeVar, Generic, List, Any


# T = TypeVar("T")
# def first(items: List[T]) -> T:
#     return items[0]

# first_item: str = first([1, 2, 4])


class Animal():
    def __init__(self) -> None:
        self.name: str = ""

    def __str__(self) -> str:
        return self.name

class NotAnimal():
    def __init__(self) -> None:
        self.name: str = ""

    def __str__(self) -> str:
        return self.name
    
T = TypeVar("T", bound=Animal)

# class GenericClass(Generic[T]):
#     def __init__(self, value: T) -> None:
#         self.value = value

#     def get_value(self) -> T:
#         return self.value

#     def set_value(self, value: T) -> None:
#         self.value = value

# my_animal = Animal()
# my_animal.name = "david1"
# obj = GenericClass[Animal](my_animal)
# animal = obj.get_value()
# print(type(animal))
# print(animal)

class GenericClass(Generic[T]):
    def __init__(self) -> None:
        self.value: T = type("T", (), {})()
        print(type(self.value))
        if isinstance(self.value, Animal):
            self.value.name = "animal"
        else:
            self.value.name = "not animal"

    def get_value(self) -> T:
        return self.value

    def set_value(self, value: T) -> None:
        self.value = value

obj = GenericClass[Animal]()
animal = obj.get_value()
# # print(type(animal))
print(animal.name)

obj = GenericClass[NotAnimal]()
animal = obj.get_value()
# # print(type(animal))
print(animal.name)
