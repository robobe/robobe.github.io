from typing import List, Generic, TypeVar

T = TypeVar("T")

class Items(Generic[T]):
    def __init__(self) -> None:
        self.__data: List[T] = []

    def add_item(self, item: T) -> None:
        self.__data.append(item)

    def first(self) -> T | None:
        if self.__data:
            return self.__data[0]
        return None


if __name__ == "__main__":
    items = Items[str]()
    items.add_item("a")

    other_items = Items[int]()
    other_items.add_item(1)