std_msgs/Header header		            # timestamp & id of coordinate system
uint64 id					            # ID of gp (seq. number: 0, 1, 2, 3, …)
uint8[] gp_type				            # list of GP type
uint8[] etype					        # list of edge type to GP
geometry_msgs/Point[] position		    # list of GP position
geometry_msgs/Quaternion[] orientation	# list of desired orientation at GP
bool is_absolute true				    # absolute or relative coordinate

# GP type
uint8 VIA_GP=0				        # GP that can be passed through without motion stop
uint8 TRANSITION_GP=1		        # The robot need to stop & mode change at the GP
uint8 GOAL_GP=2				        # final goal GP

# etype
uint8 EDGE_BASIC=0			        # edge type is not specified (or unknown)
uint8 EDGE_SIDEWALK=1			    # sidewalk of vehicle road
uint8 EDGE_ROAD=2			        # general road (indoor corridor, outdoor mixed road, outdoor walkway)
uint8 EDGE_CROSSWALK=3		        # pedestrian crosswalk
uint8 EDGE_ELEVATOR=4			    # elevator doorway
uint8 EDGE_ESCALATOR=5			    # escalator
uint8 EDGE_STAIR=6			        # stair
uint8 EDGE_DOORWAY=10			    # general doorway
uint8 EDGE_SUBWAY_GATE=11           # subway entrance/exit
uint8 EDGE_SUBWAY_TICKET_GATE=12    # subway ticket gate
uint8 EDGE_SUBWAY_SCREEN_DOOR=13    # subway screen door