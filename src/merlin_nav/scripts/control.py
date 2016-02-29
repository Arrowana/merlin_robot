#!/usr/bin/python
import math

def cap_angle_deg(angle):
    """Return angle capped to [-180;180]"""
    angle=angle % 360
    angle=(angle+360) % 360
    if(angle>180):
        angle-=360
    return angle

def cap_angle(angle):
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < -math.pi:
        angle += 2*math.pi
    
    return angle

if __name__ == "__main__":
    for a in [x*math.pi/25 for x in range(100)]:
        print a, cap_angle(a)
