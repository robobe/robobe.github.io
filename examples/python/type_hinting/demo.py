from typing import List
from typing import Union
from typing import Dict

number = Union[int, float]

def get_list(item: number) -> List[number]:
    list = [1, 2, item]
    return list


def build_map() -> Dict[str, number]:
    return {
        1: "a",
        "a": 1
    }
print(get_list(1))
