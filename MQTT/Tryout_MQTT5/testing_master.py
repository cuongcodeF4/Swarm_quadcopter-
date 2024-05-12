import math

def distance(lat1, lon1, lat2, lon2):
        R = 6371.0  
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R *c*1000    # uint m
        return distance


# two GPS coordinate
GPS_1 = {
    "LAT" : 123,
    "LON" : 456
}
GPS_2 = {
    "LAT" : 345,
    "LON" : 678
}
yaw_angle = None

#GET DISTANCE
hypotenuse = distance(GPS_1['LAT'], GPS_1["LON"],GPS_2["LAT"], GPS_2["LON"])

#let say that the first drone is the origin
#khoang cach theo LAT
temp_GPS_1 = {
        "LAT" : GPS_1["LAT"],
        "LON" : GPS_2["LON"]
}
#distance cal
LAT_dis = distance(temp_GPS_1['LAT'], temp_GPS_1["LON"], GPS_2["LAT"], GPS_2["LON"])

#khoanng cach thao LON
temp_GPS_1 = {
        "LAT" : GPS_2["LAT"],
        "LON" : GPS_1["LON"]
}
#ditance cal
LON_dis = distance(temp_GPS_1['LAT'], temp_GPS_1["LON"], GPS_2["LAT"], GPS_2["LON"])

#scan to see which quater the second drone belong
if abs(GPS_2["LON"]) > abs(GPS_1["LON"]):
    #it's on the upper side
    if abs(GPS_2["LAT"]) > abs(GPS_1["LAT"]):
        #it's on the right side
        quater = 2
    else:
        quater = 1
        #it's on the left side
else:
    #it's on the down side
    if abs(GPS_2["LAT"]) > abs(GPS_1["LAT"]):
        #it's on the right side
        quater = 3
    else:
        quater = 4
        #it's on the left side    

#output the yaw angle
if quater == 1:
    yaw_angle = math.atan(LON_dis/LAT_dis)
elif quater == 2:
    yaw_angle = -math.atan(LON_dis/LAT_dis)
elif quater == 3:
    yaw_angle = -(90 + (90 - math.atan(LON_dis/LAT_dis)))
else:
    yaw_angle = 90 + (90 - math.atan(LON_dis/LAT_dis))