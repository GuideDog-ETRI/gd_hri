import cv2
import numpy as np
import time

# Create mapping index from ERP image to Rectilinear image
# @param erp_W: width of input erp image
# @param erp_H: height of input erp image
# @param theta: the direction in which the ERP image is projected; back:0, left: pi/2, front: pi, right: 1.5pi  [Unit: Rad]
# @param hfov: horizontal field of view angle of the projected rectilinear image [Unit: Rad]
# @param vfov: vertical field of view angle of the projected rectilinear image [Unit: Rad]
# @ret erp_xi, erp_yi: mapping indeces (output resolution is determined automatically relative to input ERP image)
def create_erp_to_rect_mapping(erp_W, erp_H, theta, hfov, vfov):
    f = erp_W / (2 * np.pi)
    
    # Rectilinear image dimensions
    rect_H = int(round(2 * f * np.tan(vfov / 2)))
    rect_W = int(round(2 * f * np.tan(hfov / 2)))
    rect_cx = rect_W / 2
    rect_cy = rect_H / 2
    
    # Create grid for rectilinear image
    rect_x, rect_y = np.meshgrid(np.arange(rect_W), np.arange(rect_H))
    
    # Calculate angles for rectilinear projection
    xth = np.arctan((rect_x - rect_cx) / f)  # relative angle from center
    xth_erp = theta + xth                    # absolute angle in erp
    
    yf = f / np.cos(xth)
    yth = np.arctan((rect_y - rect_cy) / yf)
    
    # Map angles to equirectangular projection coordinates
    erp_x = (xth_erp * erp_W / (2 * np.pi)) % erp_W  # wrap around
    erp_y = yth * erp_H / np.pi + erp_H / 2

    # Use nearest neighbor interpolation for pixel mapping
    erp_xi = np.clip(np.round(erp_x).astype(int), 0, erp_W - 1)
    erp_yi = np.clip(np.round(erp_y).astype(int), 0, erp_H - 1)
    
    return erp_xi, erp_yi

# Project ERP image to Rectilinear image usign pre-computed mapping
# @param erp_image: input erp image
# @param erp_xi: mapping x-indeces
# @param erp_yi: mapping y-indeces
# @ret rect_image: rectilinear projection image
def erp_to_rect(erp_image, erp_xi, erp_yi):
    rect_image = erp_image[erp_yi, erp_xi]
    return rect_image

# Main function
if __name__ == '__main__':
    # Read the image
    image = cv2.imread('erp_image.png')
    if image is None:
        print('Image not found!')
        exit(-1)

    cv2.namedWindow('ERP image', 0)
    cv2.imshow('ERP image', image)
    cv2.waitKey(1)

    # Rectilinear projection of ERP image
    theta = np.radians(180)     # direction of projection
    hfov = np.radians(120)      # horizontal fov angle
    vfov = np.radians(90)       # vertical fov angle
    erp_H, erp_W, channels = image.shape
    erp_xi, erp_yi = create_erp_to_rect_mapping(erp_W, erp_H, theta, hfov, vfov)
    start_time = time.time()
    rect_image = erp_to_rect(image, erp_xi, erp_yi)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time:.3f} sec")
    cv2.imshow('rectilinear projection image', rect_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()