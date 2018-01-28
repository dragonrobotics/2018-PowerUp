# these are for the 4 ultrasonic sensors on the front(f), left(l), right(r) and
# top(t) which is the sensor on the lift.
I2C_ADDRESS_F = 92
I2C_ADDRESS_R = 94
I2C_ADDRESS_L = 96
I2C_ADDRESS_T = 98

# These will be constant for all the sensors. The READ_OFFSET is what we add to
# the even address to get the odd. The READ_SENSOR is the numbers we send to
# the sensors.
READ_OFFSET = 1
READ_SENSOR = 81

# Each of these are the keys for each of the sensors to change the address.
# One has to be even and one has to be odd.
ADDR_UNLOCK_F1 = 170
ADDR_UNLOCK_F2 = 165

ADDR_UNLOCK_R1 = 180
ADDR_UNLOCK_R2 = 167

ADDR_UNLOCK_L1 = 160
ADDR_UNLOCK_L2 = 163

ADDR_UNLOCK_T1 = 172
ADDR_UNLOCK_T2 = 161
