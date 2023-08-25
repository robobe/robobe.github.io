from typing import TypedDict

class Movie(TypedDict, total=False):
    name: str
    year: int

movie: Movie = {"name": "toy"}
d = dict(movie)
print(type({}))
print(type(movie))