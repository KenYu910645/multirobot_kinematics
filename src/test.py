import math 
def normalize_angle(angle):
    '''
    [-pi, pi]
    '''
    sign = None 
    if angle >= 0:
        sign = 1
    else: 
        sign = -1 

    ans = angle % (math.pi*2 * sign)

    if ans < -math.pi: # [-2pi, -pi]
        ans += 2*math.pi
    elif ans > math.pi: # [pi, 2pi]
        ans -= 2*math.pi
    return ans