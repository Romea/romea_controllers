## Driving command or odometry msg for a FourWheelSteering vehicle.
#  $Id$

# Assumes FourWheelSteering with front-wheel and rear-wheel steering. The left
# and right front wheels are generally at different angles. To simplify,
# the commanded angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates a desired absolute
# rate of change either left or right.
#
float32 front_steering_angle           # position of the virtual angle (radians)
float32 rear_steering_angle            # position of the virtual angle (radians)
float32 front_steering_angle_velocity  # rate of change (radians/s)
float32 rear_steering_angle_velocity   # rate of change (radians/s)

# Speed and acceleration (the 1st and 2nd
# derivatives of position). All are measured at the vehicle's
# center of rotation, typically the center of the rear axle.
#
# Speed is the desired scalar magnitude of the velocity vector.
# Direction is forward unless the sign is negative, indicating reverse.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates a desired absolute
# magnitude; that includes deceleration.
#
float32 speed                   # forward speed (m/s)
float32 acceleration            # acceleration (m/s^2)
