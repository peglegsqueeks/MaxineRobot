import random
from .Condition import Condition


class RandomProbabilityCondition(Condition):
    """
    Condition based on a random probability
    Condition is met with probability P,
    Condition fails with probability 1- P
    """

    def __init__(self, p: float):
        """
        Initilaises the condition

        arguments:
            - p: the probability of condition being met
        """
        super().__init__(f"Random with chance {p}")
        self.p = p

    def condition_met(self) -> bool:
        # condition met if p less than a uniform random var
        # (true p % of the time)
        rnd = random.random()
        return self.p >= rnd
