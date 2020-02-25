def map_val(value, min1, max1, min2, max2):
    # Figure out how 'wide' each range is
    span1 = max1 - min1
    span2 = max2 - min2

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - min1) / float(span1)

    # Convert the 0-1 range into a value in the right range.
    return min2 + (valueScaled * span2)

def get_vertical_vel(r2, l2, r2_vel_adj_flag, l2_vel_adj_flag):

    r2 = map_val(r2, 1, -1, 0, 1)
    l2 = map_val(l2, 1, -1, 0, -1)

    if l2 != -0.5:
        l2_vel_adj_flag = 1
        
    if l2_vel_adj_flag == 0:
        l2 = 0

    if r2 != 0.5:
        r2_vel_adj_flag = 1
        
    if r2_vel_adj_flag == 0:
        r2 = 0

    if abs(r2) > abs(l2):
        return round(r2, 1), r2_vel_adj_flag, l2_vel_adj_flag
    else:
        return round(l2, 1), r2_vel_adj_flag, l2_vel_adj_flag

