"""
author : Thomas Demmer
topic : Class containing the tools to compute a command based on an error and a set of PID parameters
#  The command outputs the following computation
#  u(k) = _G * (e(k) + _Ki * _integralTerm(k) + _Kd * _derivateTerm(k))
"""
import time
import numpy as np

def millis():
    return int(round(time.time() * 1000))


class PidTools(object):

    def __init__(self):
        self._command = 0
        self._commandSaturation = 0
        self._responseDelay = 0

        self._G = 0
        self._Ki = 0
        self._Kd = 0
        self._maxError = -1

        self._lastError = 0
        self._integralTerm = 0
        self._derivateTerm = 0

        self._smoothingFactor = 1.0
        self._newWaypoint = True

    # self._newWPstart = 0

    def updateIntegral(self, error, sampleTime):

        """
        Update the Integral term of the discrete PID
        @info : the integration function is based on a conditional integration (no integration when command saturated)
        """

        if self._newWaypoint or self._Ki == 0:
            # self._newWPstart = millis()
            return self._integralTerm

        # if millis() - self._newWPstart < self._responseDelay :
        # return self._integralTerm

        if ((self._commandSaturation == 0 or np.abs(self._command) < self._commandSaturation) and (
                np.abs(error) < self._maxError or self._maxError == - 1)):
            # Conditionnal integration :
            # we update the integral term only if the previous command wasn't saturated
            self._integralTerm += sampleTime * error

        return self._integralTerm

    def updateDerivate(self, error, sampleTime):

        """
        Update the derivative term of the discrete PID
        @info : the cumputation uses the previous value of the error contained in the _lastError param
        """
        if self._newWaypoint or sampleTime == 0:
            self._derivateTerm = 0
        else:
            self._derivateTerm = ((error - self._lastError) / sampleTime + self._derivateTerm) / 2
        # average over the last two values

        self._lastError = error

        return self._derivateTerm

    def computeCommand(self, error, sampleTime):
        """
        Computes the output command
        @option : if activated, the command is saturated with the _commandSaturation param
        """
        self.updateIntegral(error, sampleTime)
        self.updateDerivate(error, sampleTime)
        self._newWaypoint = False
        if (self._commandSaturation != 0):
            self._command = np.max(np.min(self._smoothingFactor * self._G * (
                        error + self._Ki * self._integralTerm + self._Kd * self._derivateTerm),
                                    self._commandSaturation), -self._commandSaturation)
        else:
            self._command = self._smoothingFactor * self._G * (
                        error + self._Ki * self._integralTerm + self._Kd * self._derivateTerm)

        return self._command

    def setSmoothingFactor(self, newFactor):
        if (newFactor > 1 or newFactor < 0):
            return
        self._smoothingFactor = newFactor

    def resetPID(self):
        self._lastError = 0
        self._integralTerm = 0
        self._derivateTerm = 0
        self._command = 0
        self._newWaypoint = True
        self._newWPstart = 0
        self._newWaypoint = True

    def getPSignal(self):
        return self._lastError * self._G

    def getISignal(self):
        return self._G * self._Ki * self._integralTerm

    def getDSignal(self):
        return self._G * self._Kd * self._derivateTerm

    def setPIDparams(self, gainP, gainI, gainD, saturation):
        self._G = gainP
        self._Ki = gainI
        self._Kd = gainD
        self._commandSaturation = saturation

    def getPIDParams(self):
        return self._G, self._Ki, self._Kd, self._commandSaturation
