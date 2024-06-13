# import math

# def distance(lat1, lon1, lat2, lon2):
#         R = 6371.0  
#         dlat = math.radians(lat2 - lat1)
#         dlon = math.radians(lon2 - lon1)
#         a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
#         distance = R *c*1000    # uint m
#         return distance

# def calculate_yaw(lat1, lon1, lat2, lon2):
#         d_lon = round((lon2 - lon1),6)
#         d_lat = round((lat2 - lat1),6)
#         yaw = math.atan(d_lon/ d_lat)
#         return math.degrees(yaw)

# def calculate_yaws(lat1, lon1, lat2, lon2):
#     dLat = lat2 - lat1
#     dLon = lon2 - lon1
#     yaw = math.atan(dLon/dLat)
#     # Convert to degrees
#     yaw = math.degrees(yaw)
#     # Make sure yaw is in the range [0, 360)
#     if yaw < 0:
#         yaw += 360
#     return yaw

# # [DEBUG] List Lat [-35.3632611, -35.3630705]
# # [DEBUG] List Lon [149.16523, 149.1654636]



# print(distance(-35.36326131,149.16522865,-35.36307182,149.16546451))  
# print("[DEBUG] yaw:", calculate_yaw(-35.3632611,149.16523,-35.3631688,149.1655406))

# print("[DEBUG] yaws:", calculate_yaws(-35.3632611,149.16523,-35.3631688,149.1655406))


# # import math

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
    "LAT" : -35.3632611,
    "LON" : 149.16523
}
GPS_2 = {
    "LAT" : -35.3631688,
    "LON" : 149.1655406
}
yaw_angle = None

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

print(math.degrees(yaw_angle))



