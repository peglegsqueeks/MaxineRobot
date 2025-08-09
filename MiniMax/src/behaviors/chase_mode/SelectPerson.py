import py_trees
from py_trees.common import Status
from ...behaviors.MaxineBehavior import MaxineBehavior
from ...depth_ai.ObjectDetectionReading import ObjectDetectionReading
from ...types.MovementDirection import MovementDirection


class SelectPerson(MaxineBehavior):
    def __init__(self):
        super().__init__("Select A Person")
        self.blackboard.register_key(
            "TARGET_PERSON", access=py_trees.common.Access.WRITE
        )

    def update(self) -> Status:
        camera = self.get_robot().camera_sensor
        reading: ObjectDetectionReading = camera.get_reading()
        people = reading.get_people_locations()

        if len(people) == 0:
            return Status.FAILURE

        people = sorted(people, key=lambda person: person.spatialCoordinates.z)
        closest_person = people[0]
        self.blackboard.set("TARGET_PERSON", closest_person)

        return Status.SUCCESS
