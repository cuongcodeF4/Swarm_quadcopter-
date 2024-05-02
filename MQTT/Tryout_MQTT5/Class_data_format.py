"""
msg_recv = {
    “TYPE” : “ALL or UNIT”,
    “ALL_CMD” : {
        “CMD” : “value”,
        “SYS_REPORT” : “BAT or GPS, etc”,
        “ALT” : “alt value”,
        “LON” : “lon value”,
        “LAT” : “lat value”,
        ……
    },
    “UNIT_CMD” : {
        “CMD” : “value”,
        "CLIENT_ID" : "client_id",
        “SYS_REPORT” : “BAT or GPS, etc”,
        “ALT” : “alt value”,
        “LON” : “lon value”,
        “LAT” : “lat value”,
        ……
    }
}
"""

def DATA_ALL_TYPE(CMD, SYS_REPORT, ALT, LON, LAT):    
    return {
        "TYPE" : "ALL",
        "ALL_CMD" : {
            "CMD" : CMD,
            "SYS_REPORT" : SYS_REPORT,
            "ALT" : ALT,
            "LON" : LON,
            "LAT" : LAT,
        }
    }


class DATA_UNIT_TYPE():
    def __init__(self, CMD):
        self.CMD = CMD
    def drone(self, SYS_REPORT, ALT, LON, LAT):
        return {
            "SYS_REPORT" : SYS_REPORT,
            "ALT" : ALT,
            "LON" : LON,
            "LAT" : LAT,
        }
    def msg(self, UNIT_ENABLE, CLIENT_DATA_PACK):
        #fail check for the data
        #both UNIT_ENABLE and CLIENT_DATA_PACK or list
        if len(UNIT_ENABLE) == len(CLIENT_DATA_PACK) and UNIT_ENABLE:
            MSG = {
                "CMD" : self.CMD,
                "UNIT_ENABLE" : UNIT_ENABLE,
            }
            
            for data in CLIENT_DATA_PACK:
                for ID in UNIT_ENABLE:
                    #add in items to the dict above
                    MSG[str(ID)] = data
        else:
            print("MISSING DATA FOR DRONE")
        return {
            "TYPE" : "UNIT",
            "UNIT_CMD" : MSG,
        }



#when deploy the data
msg_1  = DATA_ALL_TYPE(
    CMD= "SYSTEM_REPORT",
    SYS_REPORT= "BAT",
    ALT=None,
    LON=None,
    LAT=None,
)
print("First form of data test: ")
print(msg_1)


#let say that we try to control 3 drone that have ID 1111, 2222 and 3333

msg_2 = DATA_UNIT_TYPE("TAKE_OFF")

#create 3 drone instance
drone_1 = msg_2.drone(
    SYS_REPORT=None,
    ALT=11,
    LON=None,
    LAT=None,
)
drone_2 = msg_2.drone(
    SYS_REPORT=None,
    ALT=22,
    LON=None,
    LAT=None,
)
drone_3 = msg_2.drone(
    SYS_REPORT=None,
    ALT=33,
    LON=None,
    LAT=None,
)

#make the final data
msg_2_2 = msg_2.msg(
    UNIT_ENABLE=[1111,2222,3333],
    CLIENT_DATA_PACK = [drone_1, drone_2, drone_3]
)

print("Second form of data pack")
print(msg_2_2)