import angle_interface as ai


class JointSpaceTargetExample(object):
    def __init__(self):
        self.target1 = ai.degree2radian([ 0,   0,  0,  0,   0,  0,  0,   0,  0])
        self.target2 = ai.degree2radian([ 0, -45,  0,  0, -45,  0,  0, -45,  0])
        self.target3 = ai.degree2radian([ 0, -45, 60,  0, -45, 60,  0, -45, 60])
        self.target4 = ai.degree2radian([20, -45, 60, 20, -45, 60, 20, -45, 60])
        self.target5 = ai.degree2radian([20,   0, 20, 20,   0, 20, 20,   0, 20])

