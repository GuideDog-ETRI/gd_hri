# System Mode (GDI -> ALL)
std_msgs/msg/Header header		# timestamp & id of coordinate system
uint8 system_mode
	uint8 STOP=0			# harness-gripped but stop state
	uint8 WALK=1			# harness-gripped and moving state
	uint8 READY=2			# power-on and sitting state
	uint8 STANDBY=3			# stand-up but harness-set-free state
	uint8 ELEVATOR=30		# 'go into elevator' mode
	uint8 ESCALATOR_UP=40		# 'ride the up-escalator' mode
	uint8 ESCALATOR_DOWN=41	# 'ride the down-escalator' mode
	uint8 STAIRS_UP=50		# 'go stairs up' mode
	uint8 STAIRS_DOWN=51		# 'go stairs down' mode
	uint8 VTR=60			# VTR teaching mode
	uint8 MANUAL=70			# full manual control mode. only GDQ works
	uint8 EMG_UNKNOWN=99		# emergency stop mode by unknown error
	uint8 EMG_GP_GEN=100		# emergency stop mode by GP generation error
	uint8 EMG_GP_ARRIVAL=101		# emergency stop mode by GP arrival error
	uint8 EMG_POSITION=102		# emergency stop mode by localization error
	uint8 EMG_OOP=103		# emergency stop mode by out of path
	uint8 EMG_LPP_GEN=104		# emergency stop mode by LPP generation error
	uint8 EMG_GP_REACH=105		# emergency stop mode by unreachable GP error
	uint8 EMG_COLLISION=106		# emergency stop mode by unexpected moving obstacle
	uint8 EMG_SPACE=107		# emergency stop mode by traversable space error
	uint8 EMG_LBW=108		# emergency stop mode by low battery warning
	uint8 EMG_LB=109			# emergency stop mode by low battery
	uint8 EMG_STUCK=110		# emergency stop mode by stuck
	uint8 EMG_MOTOR=111		# emergency stop mode by motor error
	uint8 EMG_SENSOR=112		# emergency stop mode by sensor error
	uint8 EMG_HARNESS=113		# emergency stop mode by harness error
	uint8 EMG_BTN=114		# emergency stop mode by emergency button