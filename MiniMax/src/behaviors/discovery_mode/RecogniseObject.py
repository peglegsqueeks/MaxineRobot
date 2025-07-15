import py_trees
from py_trees.common import Status
from ...behaviors.MaxineBehavior import MaxineBehavior
from ...depth_ai.ObjectDetectionReading import ObjectDetectionReading
from ...types.MovementDirection import MovementDirection
from ...depth_ai.ObjectDetectionPipeline import ObjectDetectionPipeline
import numpy as np

class RecogniseObject(MaxineBehavior):
    def __init__(self):
        super().__init__("Select an object")
        global old_things, update_count
        self.blackboard.register_key("TARGET_OBJECT", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("UPDATE_COUNTER", access=py_trees.common.Access.WRITE)
        #print("hello")
        closest_object="test"
        old_things=[]
        update_count=1
        self.blackboard.set("TARGET_OBJECT", closest_object)
        self.blackboard.set("UPDATE_COUNTER", update_count)

    def update(self) -> Status:
        global old_things, update_count
        camera = self.get_robot().camera_sensor
        reading: ObjectDetectionReading = camera.get_reading()
        things = reading.get_people_locations()
        
        if len(things) == 0:
            #print("no objects")
            return Status.FAILURE
        else:
            #print(things)
            old_things.append(things)
            #print(old_things)
            new_list=[]
            for one_object in old_things:
                if one_object not in new_list:
                    new_list.append(one_object)
            #print("Reduced List" + str(new_list))
            self.blackboard.set("TARGET_OBJECT", new_list)
            if len(old_things) >6:
                old_things=[]
        update_count=self.blackboard.get("UPDATE_COUNTER") 
        update_count=update_count+1   
        if update_count>20:
            update_count=1
        self.blackboard.set("UPDATE_COUNTER", update_count)    
            
    
            
        #things = sorted(things, key=lambda person: person.spatialCoordinates.z)
        #closest_object = things[0]
        #print(closest_object)
        
        return Status.SUCCESS
