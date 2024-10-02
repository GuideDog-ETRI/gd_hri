# user command constant
uint8 GO=0                  # 'go' command 
uint8 STOP=1               # 'stop' command
uint8 RISE=2                # 'rise' command
uint8 SIT=3                  # 'sit' command
uint8 READY_TO_GO=4     # 'ready' command
uint8 TURN=5               # 'turn' command
uint8 FIND_DOOR=10            # command for finding door
uint8 FIND_ELEVATOR=11       # command for finding elevator
uint8 FIND_STAIRS=12          # command for finding stairs
uint8 FIND_ESCALATOR=13       # command for finding escalator
uint8 FIND_CROSSWALK=14       # command for finding crosswalk
uint8 FIND_SUBWAY_GATE=15    # command for finding subway gate
uint8 FIND_SUBWAY_TICKET_GATE=16       # command for finding subway ticket gate
uint8 FIND_PLATFORM=17            # command for finding boarding platform
uint8 FIND_SCREEN_DOOR=18       # command for finding screen door

uint8 usr_cmd

# parameter for user command
int32 NONE=0	       # no parameter needed
int32 RIGHT_HALF=1     # 90 degree turn
int32 LEFT_HALF=-1      # -90 degree turn
int32 RIGHT_FULL=2      # 180 degree turn
int32 LEFT_FULL=-2       # -180 degree turn

int32 cmd_param                      # Any integer except -2~2 can be used for turning angle