std_msgs/Header header           # timestamp
uint8 errcode 0                  # error code
uint8 ERR_NONE=0                 # OK (no error)
uint8 ERR_NONE_AND_FIND_SUCCESS=0
uint8 ERR_NONE_AND_FIND_FAILURE=1
uint8 ERR_UNKNOWN=2              # unknown error / general error
uint8 ERR_NOT_INIT_MODEL=3
uint8 ERR_NO_IMAGE=4