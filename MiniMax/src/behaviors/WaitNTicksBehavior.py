
import py_trees
from py_trees.common import Status
from src.behaviors.MaxineBehavior import MaxineBehavior
from py_trees.behaviour import Behaviour
from uuid import uuid4

class WaitNTicksBehavior(MaxineBehavior):
    def __init__(self, n: int):
        super().__init__(f"Wait {n} ticks")
        self.n = n
        self.id = uuid4()

        # register use of the TARGET_PERSON on the blackboard
        self.blackboard.register_key(
            f"N-{self.id}", access=py_trees.common.Access.WRITE
        )
        self.blackboard.set(f"N-{self.id}",0)

    def update(self) -> Status:
        current_n = self.blackboard.get(f"N-{self.id}")
        
        if current_n < self.n:
            self.blackboard.set(f"N-{self.id}",current_n + 1)
            return Status.FAILURE
        
        self.blackboard.set(f"N-{self.id}",0)
        return Status.SUCCESS
        

