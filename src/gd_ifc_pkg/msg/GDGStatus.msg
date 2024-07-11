std_msgs/Header header           # timestamp
uint8 errcode 0                  # error code
uint8 ERR_NONE=0                 # OK (no error)
uint8 ERR_UNKNOWN=1              # unknown error / general error
uint8 ERR_GP_GEN=20              # GP generation failure
uint8 ERR_GP_ARRIVAL=21          # GP arrival failure
uint8 ERR_LOCALIZATION=22        # localization failure (lost)
uint8 ERR_OOP=23                 # out of path