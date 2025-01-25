import math

class Vec2f:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Vec2f(x={self.x}, y={self.y})"

    def __add__(self, other):
        return Vec2f(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2f(self.x - other.x, self.y - other.y)

    def length(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)
