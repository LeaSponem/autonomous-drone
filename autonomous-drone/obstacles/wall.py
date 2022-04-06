from obstacle import Obstacle
import numpy as np


class WallObstacle(Obstacle):
    def __init__(self, x_origin, y_origin, dimension, angle):
        Obstacle.__init__(self, x_origin, y_origin)
        self._dimension = dimension
        self._angle = angle

    def get_obstacle_origin(self):
        return self._x_origin, self._y_origin

    def get_obstacle_position(self):
        x_0, y_0 = self.get_obstacle_origin()
        x = self._dimension*np.cos(self._angle)
        y = self._dimension * np.sin(self._angle)
        return x_0+x,y_0+y

    def obstacle_equation(self, x):
        x_0, y_0 = self.get_obstacle_origin()
        x_f, y_f = self.get_obstacle_position()
        a = (y_f - y_0)/(x_f - x_0)
        b = (y_0*x_f - y_f*x_0)/(x_f - x_0)
        return a*x+b
