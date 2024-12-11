#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from pick_and_place_behaviours import *
from reactive_sequence import RSequence

class PickAndPlaceBehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising pick and place behaviour tree")

        # Create behaviors
        detect_cube = DetectCube()
        
        # Navigation behaviors
        nav_to_pick = NavigateToPosition("pick position", -1.1480, -6.1, -0.709)
        nav_to_place = NavigateToPosition("place position", 2.6009, -1.7615, 0.0)
        nav_to_start = NavigateToPosition("start position", -1.1480, -6.1, -0.709)
        
        # Pick and place behaviors
        pick_cube = PickCube()
        place_cube = PlaceCube()

        # Create the main task sequence
        main_sequence = RSequence(
            name="Pick and Place Task",
            children=[
                # 1. Detect cube
                detect_cube,
                
                # 2. Navigate to pick position and pick
                nav_to_pick,
                pick_cube,
                
                # 3. Carry cube to second table
                nav_to_place,
                
                # 4. Place cube
                place_cube,
                
                # 5. Check result and retry if needed
                pt.decorators.FailureIsRunning(
                    child=pt.composites.Selector(
                        name="Task Completion Check",
                        children=[
                            place_cube,  # Try placing again
                            RSequence(   # If placing fails, return to start and retry
                                name="Return and Retry",
                                children=[
                                    nav_to_start,
                                    detect_cube,
                                    nav_to_pick,
                                    pick_cube,
                                    nav_to_place,
                                    place_cube
                                ]
                            )
                        ]
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