from elliptec.elliptec_ell14_with_speed import ElliptecRotator
from elliptec.elliptec_speed_serial import set_speed

set_speed(port="COM6", percent=10)
rot = ElliptecRotator(max_address="0")

rot.set_speed(60)
