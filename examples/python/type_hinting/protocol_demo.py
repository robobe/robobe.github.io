# pylint: disable=missing-module-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=missing-function-docstring
from typing import Iterable
from typing_extensions import Protocol


class SupportClose(Protocol):
    def close(self) -> None:
        pass


class Resource:
    def close(self) -> None:
        pass


class ResourceWithoutClose:
    pass

resource: SupportClose = Resource()
resource2: SupportClose = ResourceWithoutClose()

def close_all(items: Iterable[SupportClose]) -> None:
    for item in items:
        item.close()


close_all([Resource(), ResourceWithoutClose()])

