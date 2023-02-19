#region VEXcode Generated Robot Configuration
from math import *
import random
from vexcode_vrc import *
from vexcode_vrc.events import get_Task_func
  
# constructors

drivetrain = Drivetrain()
brain = Brain()
bottom_distance = Distance("BottomDistance", 18)
roller_optical = Optical("RollerOptical", 2)
gps = GPS("GPS", 3)
intake_motor_group = Motor("IntakeMotorGroup", 10)
bottom_line_tracker = LineTracker("BottomLineTracker", 22)
middle_line_tracker = LineTracker("MiddleLineTracker", 23)
top_line_tracker = LineTracker("TopLineTracker", 24)
#endregion VEXcode Generated Robot Configuration

#region constants
BLUE_HIGH_GOAL_COORDINATE = [-1307, -1307]
RED_HIGH_GOAL_COORDINATE  = [ 1307,  1307]
#endregion constants

#region helper functions
def format_angle(a):
    """Map an arbitary angle to -180 to 180 degrees format"""
    # check whether the angle is positive
    sign = 1
    if a < 0:
        sign = -1
    else:
        sign = 1
    
    positive_a = abs(a)

    # eliminate coterminal angles greater than 360 degrees
    mod = positive_a % 360

    if (mod <= 180):
        return sign * mod
    else:
        return sign * (mod - 360)

#endregion helper functions

#region class definitions
class Coordinates:
    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

class Chassis:
    __instance__ = None

    def __init__(self):
        if Chassis.__instance__ is None:
           Chassis.__instance__ = self
        else:
           raise Exception("You cannot create another Chassis class")

    def face_angle(self, angle):
        """Orients the robot to a specific absolute orientation"""
        drivetrain.set_turn_velocity(100,PERCENT)
        position = Coordinates(gps.x_position(MM), gps.y_position(MM), gps.heading())

        # calculates the angle of rotation
        target_angle = format_angle(angle - position.theta)
        drivetrain.turn_for(RIGHT, target_angle, DEGREES)

    def face_coordinate(self, x, y, aiming, offset):
        """Rotates the robot to face a specific coordinate on the field"""
        drivetrain.set_turn_velocity(100,PERCENT)
        position = Coordinates(gps.x_position(MM), gps.y_position(MM), gps.heading())

        # relative coordinate difference between the point and the robot's position
        x_dist = x - position.x
        y_dist = y - position.y

        # absolute orientation to face a point
        target_angle = 90 - atan2(y_dist, x_dist) * 180 / pi

        if aiming:
            # face the coordinate backwards
            drivetrain.turn_for(RIGHT, float(format_angle(180 + target_angle - position.theta + offset)), DEGREES)
        else:
            # face the coordinate
            drivetrain.turn_for(RIGHT, float(format_angle(target_angle - position.theta + offset)), DEGREES)

    def move_to_point(self, x, y):
        """Moves the robot to a specific coordinate on the field"""
        drivetrain.set_drive_velocity(100, PERCENT)
        position = Coordinates(gps.x_position(MM), gps.y_position(MM), gps.heading())

        # relative coordinate difference between the point and the robot's position
        x_dist = x - position.x
        y_dist = y - position.y

        # calculate the distance to the point
        dist = sqrt(x_dist**2 + y_dist**2)

        # absolute orientation to face a point
        target_angle = 90 - atan2(y_dist, x_dist) * 180 / pi

        # face the coordinate
        drivetrain.turn_for(RIGHT, float(format_angle(target_angle - position.theta)), DEGREES)
        
        # move to the point
        drivetrain.drive_for(FORWARD, dist, MM)

    def move_to_point_backwards(self, x, y):
        """Moves the robot backwards to a specific coordinate on the field"""
        drivetrain.set_drive_velocity(100, PERCENT)
        position = Coordinates(gps.x_position(MM), gps.y_position(MM), gps.heading())

        # relative coordinate difference between the point and the robot's position
        x_dist = x - position.x
        y_dist = y - position.y

        # distance to the point
        dist = sqrt(x_dist**2 + y_dist**2)

        # absolute orientation to face a point
        target_angle = 90 - atan2(y_dist, x_dist) * 180 / pi

        # face the coordinate backwards
        drivetrain.turn_for(RIGHT, float(format_angle(180 + target_angle - position.theta)), DEGREES)
        
        # move backwards to the point
        drivetrain.drive_for(REVERSE, dist, MM)

class Roller:
    __instance__ = None

    def __init__(self):
        if Roller.__instance__ is None:
           Roller.__instance__ = self
        else:
           raise Exception("You cannot create another Roller class")

    def score(self):
        """Score the roller in front of the robot"""
        drivetrain.drive(REVERSE)

        # Once the roller is detected, drive closer to make contact
        while not roller_optical.is_near_object():
            wait(5, MSEC)
        drivetrain.stop()
        drivetrain.drive_for(REVERSE, 35, MM)

        # Spin the intake to score the roller
        intake_motor_group.set_velocity(100, PERCENT)
        intake_motor_group.spin_for(FORWARD, 34, DEGREES)

class Robot:
    __instance__ = None

    def __init__(self):
        if Robot.__instance__ is None:
            self.chassis = Chassis()
            self.roller  = Roller()
            Robot.__instance__ = self
        else:
           raise Exception("You cannot create another Robot class")

    @staticmethod
    def face_angle(angle):
        """Orients the robot to a specific absolute orientation"""
        Robot.__instance__.chassis.face_angle(angle)
    
    @staticmethod
    def face_coordinate(x, y, aiming = False, offset = 0):
        """Rotates the robot to face a specific coordinate on the field"""
        Robot.__instance__.chassis.face_coordinate(x, y, aiming, offset)
    
    @staticmethod
    def move_to_point(x, y):
        """Moves the robot to a specific coordinate on the field"""
        Robot.__instance__.chassis.move_to_point(x, y)
    
    @staticmethod
    def move_to_point_backwards(x, y):
        """Moves the robot backwards to a specific coordinate on the field"""
        Robot.__instance__.chassis.move_to_point_backwards(x, y)
    
    @staticmethod
    def score_roller():
        """Score the roller in front of the robot"""
        Robot.__instance__.roller.score()

    @staticmethod
    def shoot_disk(degrees):
        """Rotate the intake to shoot disks"""
        intake_motor_group.spin_for(REVERSE, degrees, DEGREES)
        intake_motor_group.spin(REVERSE)

        # wait until all disks are shot
        while top_line_tracker.reflectivity(PERCENT) > 80 or \
            middle_line_tracker.reflectivity(PERCENT) > 80 or \
            bottom_line_tracker.reflectivity(PERCENT) > 80:
            wait(5, MSEC)
        intake_motor_group.stop()
    
#endregion class definitions



# ----------------------------------------------------------------------------
#
#    Project:           14683A VR skills
#    Description:       The source code for 14683A's VR skills
#                       
#    Starting Position: A
#    Preload:           2
#
# ----------------------------------------------------------------------------

robot = Robot()

def main():
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100,PERCENT)
    intake_motor_group.set_velocity(100, PERCENT)

    # ------------------------------------------------------------------------
    # Blue half field route
    # ------------------------------------------------------------------------

    # shoot out first three disks
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-2)
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(-1450, 1260)
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-2)
    robot.shoot_disk(degrees=110)
    
    # score one roller
    robot.face_angle(90)
    robot.score_roller()

    # score one roller
    drivetrain.drive_for(FORWARD, 20, MM)
    robot.move_to_point(-900, 1475)
    robot.face_angle(180)
    robot.score_roller()

    # intake three stacked disks (move at the same time)
    robot.move_to_point(-900, 1120)
    drivetrain.set_drive_velocity(17, PERCENT)
    drivetrain.drive(FORWARD)
    intake_motor_group.spin_for(REVERSE, 100, DEGREES)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)

    # shoot out three disks (move at the same time)
    intake_motor_group.set_velocity(30, PERCENT)
    intake_motor_group.spin(REVERSE)
    robot.face_coordinate(RED_HIGH_GOAL_COORDINATE[0], RED_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-4.6)
    intake_motor_group.set_velocity(90, PERCENT)
    drivetrain.set_drive_velocity(40, PERCENT)
    drivetrain.drive_for(REVERSE, 400, MM)
    intake_motor_group.stop()
    intake_motor_group.set_velocity(100, PERCENT)

    # intake and shoot out three disks (move at the same time)
    intake_motor_group.set_velocity(80, PERCENT)
    intake_motor_group.spin(REVERSE)    
    robot.move_to_point(298, 290)
    intake_motor_group.set_velocity(1.5, PERCENT)
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-2)
    intake_motor_group.set_velocity(90, PERCENT)
    drivetrain.set_drive_velocity(19, PERCENT)
    drivetrain.drive(FORWARD)
    robot.shoot_disk(degrees=95)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)
    intake_motor_group.set_velocity(100, PERCENT)

    # intake and shoot out three blue low goal disks
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(450, 500)
    robot.move_to_point(440, 700)
    robot.move_to_point(450, 900)
    robot.move_to_point(460, 1000)
    intake_motor_group.stop()
    robot.move_to_point_backwards(420, 380)
    intake_motor_group.set_velocity(95, PERCENT)
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-1)
    robot.shoot_disk(degrees=100)

    # intake three blue low goal disks
    intake_motor_group.set_velocity(90, PERCENT)
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(520, 400)
    robot.move_to_point(750, 440)
    robot.move_to_point(950, 440)
    robot.face_angle(80)
    wait(50,MSEC)
    intake_motor_group.set_velocity(1.5, PERCENT)

    # shoot out three blue low goal disks and seven match load
    robot.move_to_point(1538, 212)
    robot.face_coordinate(RED_HIGH_GOAL_COORDINATE[0], RED_HIGH_GOAL_COORDINATE[1], aiming=True, offset=7.2)
    intake_motor_group.set_velocity(100, PERCENT)
    robot.shoot_disk(degrees=398)
    
    # intake three stacked disks (move at the same time)
    robot.move_to_point(1125, -153)
    drivetrain.set_drive_velocity(6, PERCENT)
    drivetrain.drive(FORWARD)
    intake_motor_group.spin_for(REVERSE, 85, DEGREES)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)
    intake_motor_group.stop()

    # shoot out six disks (move at the same time)
    robot.face_angle(191)
    intake_motor_group.set_velocity(92, PERCENT)
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(928, -694)
    drivetrain.set_drive_velocity(16, PERCENT)
    drivetrain.drive(FORWARD)
    robot.shoot_disk(degrees=180)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)
    intake_motor_group.set_velocity(100, PERCENT)

    # intake and shoot out two disks
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(1342, -1342)
    robot.move_to_point_backwards(1402, -1175)
    robot.face_coordinate(RED_HIGH_GOAL_COORDINATE[0], RED_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-1.5)
    robot.shoot_disk(degrees=50)
    intake_motor_group.stop()

    # score two rollers
    robot.face_angle(270)
    robot.score_roller()
    drivetrain.drive_for(FORWARD, 100, MM)
    robot.move_to_point_backwards(903, -1435)
    robot.face_angle(0)
    robot.score_roller()

    # ------------------------------------------------------------------------
    # Red half field route
    # ------------------------------------------------------------------------

    # intake three disks
    drivetrain.drive_for(FORWARD, 50, MM)
    intake_motor_group.set_velocity(70, PERCENT)
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(-308, -335)

    # shoot out three disks (move at the same time)
    intake_motor_group.set_velocity(1.5, PERCENT)
    robot.face_coordinate(RED_HIGH_GOAL_COORDINATE[0], RED_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-2)
    intake_motor_group.set_velocity(95, PERCENT)
    drivetrain.set_drive_velocity(13, PERCENT)
    drivetrain.drive(FORWARD)
    robot.shoot_disk(degrees=100)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)
    intake_motor_group.set_velocity(100, PERCENT)

    # intake three red low goal disks
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(-458, -480)
    robot.move_to_point(-480, -680)
    robot.move_to_point(-480, -1000)
    wait(20, MSEC)

    # shoot out three red low goal disks
    intake_motor_group.set_velocity(1, PERCENT)
    robot.move_to_point_backwards(-450, -400)
    robot.face_coordinate(RED_HIGH_GOAL_COORDINATE[0], RED_HIGH_GOAL_COORDINATE[1], aiming=True)
    intake_motor_group.set_velocity(98, PERCENT)
    drivetrain.set_drive_velocity(1, PERCENT)
    drivetrain.drive(FORWARD)
    robot.shoot_disk(degrees=110)
    drivetrain.stop()
    drivetrain.set_drive_velocity(100, PERCENT)

    # intake three red low goal disks
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(-508, -420)
    robot.move_to_point(-708, -440)
    robot.move_to_point(-965, -440)
    wait(50, MSEC)

    # shoot out three red low goal disks and seven match load
    intake_motor_group.set_velocity(0.5, PERCENT)
    robot.move_to_point(-1540, -208)
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=2.5)
    intake_motor_group.set_velocity(100, PERCENT)
    robot.shoot_disk(degrees=392)

    # intake and shoot out three stacked disks
    robot.move_to_point(-1126, 145)
    intake_motor_group.spin_for(REVERSE, 120, DEGREES)
    robot.move_to_point(-1173.7350199733687, 279.46071904127825);
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=-2)
    intake_motor_group.set_velocity(95, PERCENT)
    intake_motor_group.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 850, MM)
    intake_motor_group.stop()

    # ------------------------------------------------------------------------
    # Middle field route
    # ------------------------------------------------------------------------

    # intake three disks    
    robot.move_to_point(-877.2636484687084, 870.7833875511361)
    intake_motor_group.set_velocity(30, PERCENT)
    intake_motor_group.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 300, MM)
    intake_motor_group.set_velocity(100, PERCENT)
    drivetrain.drive_for(FORWARD, 610, MM)

    # move to the middle of the field
    intake_motor_group.stop()
    drivetrain.drive_for(FORWARD, 540, MM)
    intake_motor_group.set_velocity(0.1, PERCENT)
    intake_motor_group.spin(REVERSE)

    # shoot out three disks    
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=1)
    intake_motor_group.set_velocity(95, PERCENT)
    robot.shoot_disk(degrees=100)
    
    # intake two disks
    intake_motor_group.set_velocity(100, PERCENT)
    intake_motor_group.spin(REVERSE)
    robot.move_to_point(608, -608)

    # shoot out two disks
    robot.face_coordinate(BLUE_HIGH_GOAL_COORDINATE[0], BLUE_HIGH_GOAL_COORDINATE[1], aiming=True, offset=1)
    intake_motor_group.set_velocity(95, PERCENT)
    intake_motor_group.spin(REVERSE)
vr_thread(main)
