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
        "UNIT_ENABLE" : [all the unit ID]
        “CLIENT_DATA” : {
            “CMD” : “value”,
            “SYS_REPORT” : “BAT or GPS, etc”,
            “ALT” : “alt value”,
            “LON” : “lon value”,
            “LAT” : “lat value”,
        }
    }
}
"""
class DATA():
    def DATA_ALL_TYPE(self,CMD, SYS_REPORT, ALT, LON, LAT):
        if CMD == "SYSTEM_REPORT":
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
        elif CMD != "SYSTEM_REPORT":
            return {
                "TYPE" : "ALL",
                "ALL_CMD" : {
                    "CMD" : CMD,
                    "ALT" : ALT,
                    "LON" : LON,
                    "LAT" : LAT,
                }
            }
    def DATA_UNIT_TYPE(self,UNIT_ENABLE, CMD, SYS_REPORT, ALT, LON, LAT):    
        if CMD == "SYSTEM_REPORT":
            return {
                "TYPE" : "ALL",
                "UNIT_CMD" : {
                    "UNIT_ENABLE" : UNIT_ENABLE,
                    "CLIENT_DATA" : {
                        "CMD" : CMD,
                        "SYS_REPORT" : SYS_REPORT,
                        "ALT" : ALT,
                        "LON" : LON,
                        "LAT" : LAT
                    }
                }
            }
        elif CMD != "SYSTEM_REPORT":
            return {
                "TYPE" : "ALL",
                "UNIT_CMD" : {
                    "UNIT_ENABLE" : UNIT_ENABLE,
                    "CLIENT_DATA" : {
                        "CMD" : CMD,
                        "ALT" : ALT,
                        "LON" : LON,
                        "LAT" : LAT,
                    }
                }
            }



#when deploy the data
#all_type deploy
msg = DATA()

msg_1 = msg.DATA_ALL_TYPE(
    CMD="TAKE_OFF",
    SYS_REPORT= "TAKE_OFF",
    ALT= 10,
    LON=0,
    LAT=0
)
print("First form of data test: ")
print(msg_1)


#let say that we try to control 3 drone that have ID 1111, 2222 and 3333
#unit_type deploy
msg_2 = msg.DATA_UNIT_TYPE(
    UNIT_ENABLE= [1111,2222,3333],
    CMD="SYSTEM_REPORT",
    SYS_REPORT= "GPS",
    ALT=10,
    LON=10,
    LAT=10
)
print("Second form of data pack")
print(msg_2)