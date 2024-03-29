import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify obstacles
def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be below all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    # Return the binary image
    return color_select

# Identify rocks
def rock_thresh(img, rgb_lower_thresh=(100, 100, 0), rgb_upper_thresh=(210, 210, 55)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be between all three threshold values in RGB
    # lower_thresh and upper_thresh will now contain a boolean array with "True"
    # where threshold was met
    between_thresh = ((img[:,:,0] > rgb_lower_thresh[0]) & (img[:,:,0] < rgb_upper_thresh[0])) \
                & ((img[:,:,1] > rgb_lower_thresh[1]) & (img[:,:,1] < rgb_upper_thresh[1])) \
                & ((img[:,:,2] > rgb_lower_thresh[2]) & (img[:,:,2] < rgb_upper_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[between_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

def rover_coords_ex(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel, xpos, ypos


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
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
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    #
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                              [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                             ])

    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_threshed = color_thresh(warped)
    obstacle_threshed  = obstacle_thresh(warped)
    rock_threshed = rock_thresh(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image = np.zeros((160, 320, 3), dtype=np.float)
    # Rover.vision_image[:, :, 0] = obstacle_threshed*255
    # Rover.vision_image[:, :, 1] = rock_threshed*255
    # Rover.vision_image[:, :, 2] = navigable_threshed
    Rover.vision_image[:, :, :] = warped

    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix, navigable_xpos, navigable_ypos = rover_coords_ex(navigable_threshed)
    obstacle_xpix, obstacle_ypix = rover_coords(obstacle_threshed)
    rock_xpix, rock_ypix = rover_coords(rock_threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    (rover_xpos, rover_ypos) = Rover.pos
    rover_yaw  = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix,
                                                        rover_xpos, rover_ypos, rover_yaw,
                                                        world_size, scale)

    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix,
                                                      rover_xpos, rover_ypos, rover_yaw,
                                                      world_size, scale)

    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix,
                                              rover_xpos, rover_ypos, rover_yaw,
                                              world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    roll_pitch_lower_limit = 0.8
    roll_pitch_upper_limit = 360 - roll_pitch_lower_limit
    #
    # roll & pitch should be less than lower limit or large than upper limit
    #
    if (Rover.roll < roll_pitch_lower_limit or Rover.roll > roll_pitch_upper_limit) \
            and (Rover.pitch < roll_pitch_lower_limit or Rover.pitch > roll_pitch_upper_limit):
        #
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1


    # 8) Convert rover-centric pixel positions to polar coordinates
    nav_dists, nav_angles = to_polar_coords(navigable_xpix, navigable_ypix)
    rock_dists, rock_angles = to_polar_coords(rock_xpix, rock_ypix)

    nav_visited = np.clip(Rover.worldmap[navigable_y_world, navigable_x_world, 2], 1, 1000)
    nav_visit_time = Rover.visit_map[navigable_y_world, navigable_x_world, 0]

    if (Rover.roll < roll_pitch_lower_limit or Rover.roll > roll_pitch_upper_limit) \
            and (Rover.pitch < roll_pitch_lower_limit or Rover.pitch > roll_pitch_upper_limit):
        #
        region = nav_dists <= 20
        #
        region_x_world = navigable_x_world[region]
        region_y_world = navigable_y_world[region]
        #
        unvisited = Rover.visit_map[region_y_world, region_x_world, 0] == 1
        #
        Rover.visit_map[region_y_world[unvisited], region_x_world[unvisited], 0] = Rover.time_step

    # Update vision image with visiting time
    # 1 - indicate the area hasn't visited yet (use 1 instead of 0, since it is better for calculate 1/visiting_time
    #
    #Rover.vision_image[navigable_ypos, navigable_xpos, 2] = np.clip(nav_visited/4, 1, 255)
    if len(navigable_x_world) > 0:
        #
        # min_visiting_time = np.min(Rover.visit_map[navigable_y_world, navigable_x_world]) - 1
        #
        # Rover.vision_image[navigable_ypos, navigable_xpos, 2] = \
        #     (255 / (Rover.visit_map[navigable_y_world, navigable_x_world, 0] - min_visiting_time))
        #
        #### highligh unvisited regions
        #
        region = nav_dists <= 90
        #
        region_x_world = navigable_x_world[region]
        region_y_world = navigable_y_world[region]
        #
        unvisited = Rover.visit_map[region_y_world, region_x_world, 0] == 1
        #
        region_xpos = navigable_xpos[region]
        region_ypos = navigable_ypos[region]
        #
        Rover.vision_image[region_ypos[unvisited], region_xpos[unvisited], 0] = 0
        Rover.vision_image[region_ypos[unvisited], region_xpos[unvisited], 1] = 255
        Rover.vision_image[region_ypos[unvisited], region_xpos[unvisited], 2] = 0

        #### highlight visited heat map
        #
        # filter out visited regions
        visited = Rover.visit_map[region_y_world, region_x_world, 0] > 1
        if np.sum(visited) > 0:
            visited_heats = Rover.visit_map[region_y_world[visited], region_x_world[visited], 0]
            visited_heats_max = np.max(visited_heats)
            visited_heats_min = np.min(visited_heats)
            visited_heats_range = visited_heats_max - visited_heats_min
            if visited_heats_range <= 0:
                visited_heats = 255
            else:
                visited_heats = 255 - (255*(visited_heats - visited_heats_min)/visited_heats_range)
            #
            Rover.vision_image[region_ypos[visited], region_xpos[visited], 0] = 0
            Rover.vision_image[region_ypos[visited], region_xpos[visited], 1] = visited_heats
            Rover.vision_image[region_ypos[visited], region_xpos[visited], 2] = 255

        #### showing local regions
        #
        # region = nav_dists <= 20
        # #
        # Rover.vision_image[navigable_ypos[region], navigable_xpos[region], 0] = Rover.vision_image[navigable_ypos[region], navigable_xpos[region], 0] * 0.7
        # Rover.vision_image[navigable_ypos[region], navigable_xpos[region], 1] = Rover.vision_image[navigable_ypos[region], navigable_xpos[region], 1] * 0.7
        # Rover.vision_image[navigable_ypos[region], navigable_xpos[region], 2] = 255

        #### Show grid line
        #
        # local region
        cv2.circle(Rover.vision_image, (160, 160), 20, (255,255,255), 2)
        #
        # effective region
        cv2.circle(Rover.vision_image, (160, 160), 90, (255,255,255), 2)


    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = nav_dists
    Rover.nav_angles = nav_angles
    Rover.nav_visited = nav_visited
    Rover.nav_visit_time = nav_visit_time
    #
    Rover.rock_dists = rock_dists
    Rover.rock_angles = rock_angles


    return Rover