#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from pick_and_place_behaviours import DetectCube, PickCube, PlaceCube
from reactive_sequence import RSequence

class PickAndPlaceBehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        rospy.loginfo("Initialising pick and place behaviour tree")

        # Create behaviors
        detect_cube = DetectCube()
        pick_cube = PickCube()
        move_to_place = pt.composites.Selector(
            name="Move to place position",
            children=[
                counter(10, "At place position?"),
                go("Go to place position!", 1, 0)
            ]
        )
        place_cube = PlaceCube()
        
        # Return to start position if place fails
        return_to_start = pt.composites.Selector(
            name="Return to start",
            children=[
                counter(10, "At start?"),
                go("Return to start!", -1, 0)
            ]
        )

        # Create main sequence
        main_sequence = RSequence(
            name="Pick and Place Sequence",
            children=[
                detect_cube,
                pick_cube,
                move_to_place,
                place_cube,
                pt.decorators.FailureIsSuccess(
                    name="Place Success Check",
                    child=pt.composites.Selector(
                        name="Place Result",
                        children=[place_cube, return_to_start]
                    )
                )
            ]
        )

        # Become the tree
        super(PickAndPlaceBehaviourTree, self).__init__(main_sequence)

        # Execute the tree
        rospy.sleep(5)  # Wait for everything to initialize
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            self.tick_tock(1)

if __name__ == '__main__':
    rospy.init_node('pick_and_place_tree')
    try:
        PickAndPlaceBehaviourTree()
    except rospy.ROSInterruptException:
        pass 