
class Base2():
    def __init__(self, max) -> None:
        self.__max = max
        self.__current = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.__current > self.__max:
            raise StopIteration()
        result = 2**self.__current
        self.__current += 1
        return result

worker = Base2(2)
worker_iter = worker.__iter__()
print(next(worker_iter))
print(worker_iter.__next__())
print(next(worker_iter))
print(next(worker_iter))
# print(next(worker_iter))

# for i in worker:
#     print(i)
