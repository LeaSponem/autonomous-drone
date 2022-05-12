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

    def _get_obstacle_origin(self):
        """
        Return the obstacle origin coordinates
        """
        return self._x_origin, self._y_origin

    def _get_obstacle_coordinates(self):
        x_0, y_0 = self._get_obstacle_origin()
        return x_0 + self._dimension * np.cos((np.pi/180)*self.angle), y_0 + self._dimension * np.sin((np.pi/180)*self.angle)

    def _get_center_coordinates(self):
        x_0, y_0 = self._get_obstacle_origin()
        x_f, y_f = self._get_obstacle_coordinates()
        return (x_0+x_f)/2, (y_0+y_f)/2

    def _obstacle_equation(self):
        """
        Return the coeff of the obstacle equation
        """
        x_0, y_0 = self._get_obstacle_origin()
        a = np.sin((np.pi/180)*self.angle)
        b = y_0 - a*x_0
        return a, b

    def intersection(self, x_drone, y_drone, angle_drone):
        """
        Compute and return the coordinates of intersection between the drone axis and the wall
        """
        a_wall, b_wall = self._obstacle_equation()
        a_drone = np.sin(angle_drone)
        b_drone = y_drone - x_drone*np.sin(angle_drone)
        # Drone perpendicular to the X axis
        if np.abs(angle_drone - 90) < 0.5 or np.abs(angle_drone - 270) < 0.5:
            x_intersection = x_drone
            y_intersection = a_wall*x_drone + b_wall
        # Obstacle perpendicular to the X axis
        elif np.abs(self.angle - 90) < 0.5 or np.abs(self.angle - 270) < 0.5:
            x_intersection = self._get_obstacle_origin()[0]
            y_intersection = a_drone*x_intersection+b_drone
        else:
            x_intersection = (b_drone - b_wall)/(a_wall - a_drone)
            y_intersection = a_wall*x_intersection+b_wall
        return x_intersection, y_intersection

    def check_obstacle_orientation(self, x_drone, y_drone, angle_drone, x_max_range, y_max_range):
        """
        Check if the distance is measured in front of the drone
        Return True if the intersection point is in front of the drone, False otherwise
        """
        x_intersection, y_intersection = self.intersection(x_drone, y_drone, angle_drone)
        drone_vector = np.array([x_max_range-x_drone, y_max_range-y_drone])
        obstacle_vector = np.array([x_intersection-x_drone, y_intersection-y_drone])
        if np.dot(drone_vector, obstacle_vector) > 0:
            return True
        return False

    def check_obstacle_dimension(self, x_intersection, y_intersection):
        x_center, y_center = self._get_center_coordinates()
        distance = np.sqrt((x_intersection-x_center)**2 + (y_intersection-y_center)**2)
        if distance < self._dimension/2:
            return True
        return False
