std_msgs/Header header
vision_msgs/BoundingBox2D bbox  # 물체의 위치 (pixel)
uint8 obj_type          # 물체의 타입/종류 (모듈간연동데이터타입정리(안).hwp를 따르고 있음)
uint8 obj_status        # 물체의 타입에 따른 상태값 (문, 엘리베이터, 신호등, 에스컬레이터, 모듈간연동데이터타입정리(안).hwp를 따르고 있음)
string cam_id	        # camera id
int32 img_w	            # (rectified erp) 이미지의 가로 해상도 (width)
int32 img_h	            # (rectified erp) 이미지의 세로 해상도 (height)
float64 htheta	        # rectified erp 이미지의 수평방향각 (unit: radian)
float64 vtheta	        # rectified erp 이미지의 수직방향각 (unit: radian)
float64 hfov	        # rectified erp 이미지의 수평시야각 (unit: radian)
float64 vfov	        # rectified erp 이미지의 수직시야각 (unit: radian)