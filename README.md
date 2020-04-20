# Pure-Pursuit
A short project simulating pure-pursuit for differential drive robot motion control.

- Geometric path tracking
    Any controller that tracks a reference path using only the geometry of the vehicle kinematics and the reference path. A geometric path tracking controller is a type of lateral controller that ignores dynamic forces on the vehicles and assumes the no-slip conditions holds on the wheels.
    https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-geometric-lateral-control-pure-pursuit-44N7x

- Pure Pursuit Concept
    Find point on path l distance away from the robot (this is the look-ahead distance)
    Find the angle between the robot's heading and the line from the robot's xy position to the xy position on the path intersected by the look-ahead distance
    Find the instantaneous center of curvature to get the distance R of the circle formed by it
    We would like to define the arc that takes the vehicle to the look-ahead xy point

- Orthogonal Projection
    https://en.wikibooks.org/wiki/Linear_Algebra/Orthogonal_Projection_Onto_a_Line