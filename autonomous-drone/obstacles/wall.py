from obstacle import Obstacle
import numpy as np


class WallObstacle(Obstacle):
    """
    Class for a specific obstacle : wall
    A wall is defined by its origin's coordinates, length and angle with the x axis
    """
    def __init__(self, x_origin, y_origin, dimension, angle):
        Obstacle.__init__(self, x_origin, y_origin)
        self._dimension = dimension
        self.angle = angle

    def get_obstacle_origin(self):
        """
        Return the obstacle origin coordinates
        """
        return self._x_origin, self._y_origin

    def obstacle_equation(self):
        """
        Return the coeff of the obstacle equation
        """
        x_0, y_0 = self.get_obstacle_origin()
        a = np.sin(self.angle)
        b = y_0 - a*x_0
        return a, b

    def intersection(self, x_drone, y_drone, angle_drone):
        """
        Compute and return the coordinates of intersection between the drone axis and the wall
        """
        a_wall, b_wall = self.obstacle_equation()
        a_drone = np.sin(angle_drone)
        b_drone = y_drone - x_drone*np.sin(angle_drone)
        # Drone perpendicular to the X axis
        if np.abs(angle_drone - 90) < 0.5:
            x_intersection = x_drone
            y_intersection = a_wall*x_drone + b_wall
        # Obstacle perpendicular to the X axis
        elif np.abs(self.angle - 90) < 0.5:
            x_intersection = self.get_obstacle_origin()[0]
            y_intersection = a_drone*x_intersection+b_drone
        else:
            x_intersection = (b_drone - b_wall)/(a_wall - a_drone)
            y_intersection = a_wall*x_intersection+b_wall
        return x_intersection, y_intersection
