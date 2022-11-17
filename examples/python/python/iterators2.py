class Point:
    def __init__(self, x, y):
        self.x, self.y = x, y
    def __iter__(self):
        yield self.x
        yield self.y

p = Point(1, 2)
p_iter = iter(p)
print(p)
print(p_iter)

x, y = p_iter
print(x, y)

l = list(p_iter)
print(l)