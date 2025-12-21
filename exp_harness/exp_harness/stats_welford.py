# SPDX-FileCopyrightText: 2025 Yuzuki Fujita
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass
from math import sqrt

EPS = 1e-12

@dataclass
class Welford:
    """平均と分散を逐次計算するクラス（Welford法）。"""

    n: int = 0
    mean: float = 0.0
    m2: float = 0.0

    def push(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.m2 += delta * delta2

    def var(self) -> float:
        if self.n < 2:
            return 0.0
        return self.m2 / (self.n - 1)

    def std(self) -> float:
        return sqrt(max(0.0, self.var()))
