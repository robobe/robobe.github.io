from dataclasses import dataclass

@dataclass
class Animal:
    name: str

def func(animal: Animal) -> None:
    return None

a = Animal(name="12345")
func(a)