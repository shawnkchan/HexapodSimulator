import math

def coxaAngle(y, x):
    '''
    Returns the angle of the coxa joint, where positive follows the right hand curl rule wrt the coxa's z-axis

    @param  y    Desired y coordinate for end effector to reach
    @param  x   Desired x coordinate for end effector to reach

    @return coxaAngle   The angle at which the Coxa's joint should be set
    '''
    return math.atan(y / x)

def femurAngle(femurLength, tibiaLength, x, y, z):
    '''
    Returns the angle of the femur joint, where positive follows the right hand curl rule wrt the femur joint
    '''
    femurAngle = math.acos(
        (tibiaLength**2 - femurLength**2 - x**2 - y**2 - z**2) / (-2*tibiaLength * math.sqrt(x**2 + y**2 + z**2))
        ) + math.atan(
            z / math.sqrt(x **2 + y**2)
            )

    return femurAngle

def tibiaAngle(femurLength, tibiaLength, x, y, z):
    tibiaAngle = math.acos((x**2 + y**2 + z**2 - femurLength**2 - tibiaLength**2) / (2 * tibiaLength * femurLength))

    return tibiaAngle