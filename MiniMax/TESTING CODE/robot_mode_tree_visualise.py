import os
import py_trees
from src.behaviors.lidarchase import make_lidar_chase_sub_tree
from src.behaviors.utils import make_mode_sub_tree
from src.types.RobotModes import RobotMode
from src.behaviors.idle_mode import make_idle_mode_sub_tree
from src.behaviors.keyboard_mode import make_keyboard_mode_sub_tree
from src.behaviors.chase_mode import make_chase_mode_sub_tree
from unittest.mock import patch
from py_trees.common import VisibilityLevel, BlackBoxLevel
from py_trees.behaviour import Behaviour

mode_to_sub_tree = {
    RobotMode.IDLE: make_idle_mode_sub_tree(),
    RobotMode.KEYBOARD_CONTROL: make_keyboard_mode_sub_tree(),
    RobotMode.CHASE: make_chase_mode_sub_tree(),
    RobotMode.LIDARCHASE: make_lidar_chase_sub_tree(),
}


def tree_to_image(name: str, root: Behaviour, visability=VisibilityLevel.DETAIL):
    # generate images for each tree
    with patch("py_trees.console.has_unicode", return_value=False):
        py_trees.display.render_dot_tree(
            root, target_directory=IMAGE_DIR, name=name, visibility_level=visability
        )

    # render_dot_tree() generates .svg and .dot files as well, we dont need these
    images_to_delete = [
        os.path.join(IMAGE_DIR, f"{name}.svg"),
        os.path.join(IMAGE_DIR, f"{name}.dot"),
    ]
    for image in images_to_delete:
        os.remove(image)


def make_top_level_image():
    """
    Makes an image for the robot main root tree (without showing the tree for each mode)
    """
    sub_trees = []
    for robot_mode, sub_tree in mode_to_sub_tree.items():
        # change the mode's sub tree visability
        sub_tree.blackbox_level = BlackBoxLevel.BIG_PICTURE
        wrapped_sub_tree = make_mode_sub_tree(robot_mode, sub_tree)
        sub_trees.append(wrapped_sub_tree)

    # root node is a selector (will run first passing sub tree)
    # ie will run sub tree of the curent mode
    robot_root = py_trees.composites.Selector("root", memory=True, children=sub_trees)
    tree_to_image("main", robot_root, VisibilityLevel.BIG_PICTURE)


def make_mode_images():
    for mode, root in mode_to_sub_tree.items():
        image_name = f"{mode.name.lower()}_mode_tree"
        tree_to_image(image_name, root)


IMAGE_DIR = "robot_mode_images"
if __name__ == "__main__":
    os.makedirs(IMAGE_DIR, exist_ok=True)
    make_mode_images()
    make_top_level_image()
