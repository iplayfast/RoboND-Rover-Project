import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, min=(160, 160, 160),max=(256,256,256)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > min[0]) \
                & (img[:,:,1] > min[1]) \
                & (img[:,:,2] > min[2]) \
                & (img[:,:,0] < max[0]) \
                & (img[:,:,1] < max[1]) \
                & (img[:,:,2] < max[2]) \
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def navThresh(img):
    return color_thresh(img,min=(160,160,160),max=(256,256,256))

def rockThresh(img):
    return color_thresh(img,min=(110,110,0),max=(256,256,50))

def unNavThresh(img):
    return color_thresh(img,min=(0,0,0),max=(126,106,95))


# Define a function to convert from image coords to Rover coords
def Rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the Rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in Rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in Rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map Rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples    
    dst_size = 5
    bottom_offset = 6
    xpos, ypos = Rover.pos
    world_size = Rover.vision_image.shape[0]
    scale = dst_size * 4 
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])

    navigable = navThresh(Rover.img)
    navigable = perspect_transform(navigable,source,destination);

    nonNavigable = unNavThresh(Rover.img)
    nonNavigable = perspect_transform(nonNavigable,source,destination);

    rocks = rockThresh(Rover.img)
    rocks = perspect_transform(rocks,source,destination)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

  
    Rover.vision_image[:,:,0] = nonNavigable * 255;   # red
    Rover.vision_image[:,:,1] = rocks * 255;          # green
    Rover.vision_image[:,:,2] = navigable* 255;      # blue



    # 5) Convert map image pixel values to Rover-centric coords
    xpix,ypix = Rover_coords(navigable)
    xpixBad,ypixBad = Rover_coords(nonNavigable)
    rockX,rockY = Rover_coords(rocks)

    # 6) Convert Rover-centric pixel values to world coordinates
    worldx,worldy = pix_to_world(xpix, ypix, xpos,ypos, Rover.yaw, world_size, scale)
    worldxBad,worldyBad = pix_to_world(xpixBad, ypixBad, xpos,ypos, Rover.yaw, world_size, scale)

    rockWorldX,rockWorldY = pix_to_world(rockX,rockY,xpos,ypos,Rover.yaw,world_size,scale)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.roll < 0.5 or Rover.roll > 359.5) or (Rover.pitch < 0.5 or
            Rover.pitch > 359.5):
        Rover.worldmap[worldyBad.astype('int64'),worldxBad.astype('int64'),0] += 1 # not navigable
        Rover.worldmap[rockWorldY.astype('int64'),rockWorldX.astype('int64'),1] += 1 # rocks be here
        Rover.worldmap[worldy.astype('int64'),worldx.astype('int64'),2] += 1 #navigable


    # 8) Convert Rover-centric pixel positions to polar coordinates
    Rover.nav_dists,Rover.nav_angles = to_polar_coords(xpix,ypix)
    Rover.rock_dists,Rover.rock_angles = to_polar_coords(rockX,rockY)
    return Rover
