def base2(max):
    for x in range(max):
        yield 2**x



print(base2(3))
for i in base2(3):
    print(i)


g = (2**x for x in range(3))
print(g)
for i in g:
    print(i)