import numpy as np
import cv2

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # clear message field
    Rover.message = ''

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        # Update the mode
        if Rover.picking_up or Rover.near_sample:
            Rover.mode = 'pickup'

        elif is_rock_found(Rover):
            Rover.mode = 'go_to_rock'


        # Check for Rover.mode status
        if Rover.mode == 'pickup':
            pickup(Rover)

        elif Rover.mode == 'go_to_rock':
            drive_to_rock(Rover)

        elif Rover.mode == 'forward':
            drive_forward(Rover)

        elif Rover.mode == 'turn':
            drive_turn(Rover)

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            drive_stop(Rover)

        # take care the looping issue
        if abs(Rover.steer) == 15 and Rover.vel >= 1:
            if Rover.steer == Rover.looping_steer:
                # Add count
                Rover.looping_steer = Rover.steer
                Rover.looping_count += 1
            else:
                # Start to counting
                Rover.looping_steer = Rover.steer
                Rover.looping_count = 1

            looping_thresh = 80
            if Rover.looping_count > looping_thresh:
                Rover.message = 'looping issue'

                steer_offset = min(Rover.looping_count - looping_thresh, 15)

                if Rover.steer == 15:
                    Rover.steer = 15 - steer_offset
                elif Rover.steer == -15:
                    Rover.steer = -15 + steer_offset


    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # Show message in vision image
    cv2.putText(Rover.vision_image, 'Mode: ' + Rover.mode, (2, 16),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(Rover.vision_image, 'Message: ' + Rover.message, (2, 32),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    # if len(Rover.nav_dists) > 0:
    #     cv2.putText(Rover.vision_image, 'Max nav_dists: ' + str(max(Rover.nav_dists)), (2, 48),
    #                 cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)

    return Rover

def is_rock_found(Rover):
    return len(Rover.rock_angles) >= 2
    # return False

def compute_steer(Rover):

    # steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
    #
    # # angles_filters = Rover.nav_dists < 60
    # # steer_weighted = np.clip(np.average(Rover.nav_angles[angles_filters] * 180 / np.pi, \
    # #                                     weights=1 / Rover.nav_visited[angles_filters]), -15, 15)
    #
    # steer_weighted = np.clip(np.average(Rover.nav_angles * 180 / np.pi, \
    #                                     weights=1 / Rover.nav_visited), -15, 15)
    #
    # return steer*0.3 + steer_weighted*0.7

    # return compute_steer_with_unvisited_region(Rover)

    steer = 0

    region = Rover.nav_dists <= 90
    unvisited = region & (Rover.nav_visit_time == 1)

    # cv2.putText(Rover.vision_image, 'whole region size: ' + str(np.sum(region)), (2, 48),
    #             cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    #
    # cv2.putText(Rover.vision_image, 'unvisited region size: ' + str(np.sum(unvisited)), (2, 64),
    #             cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)

    if np.sum(unvisited) > 100:
        steer = compute_steer_with_unvisited_3(Rover, unvisited)
    else:
        steer = compute_steer_with_visited_region(Rover)

    return steer

def compute_steer_with_unvisited_3(Rover, unvisited):

    region_M0, region_L1, region_L2, region_R1, region_R2, region_data = compute_region_data(Rover, unvisited)

    #### Compute steering angle based on the pattern of mean angles and mean distances
    #
    if region_data[0, 3] >= 50 and region_data[1, 3] >= 55 and region_data[2, 3] >= 55 and \
        region_data[3, 3] >= 55 and region_data[4, 3] >= 55:


        Rover.mode = 'turn'
        Rover.turn_desired_yaw = Rover.yaw + (region_data[2, 2] * 180 / np.pi)

        steer = np.clip(region_data[2, 2] * 180 / np.pi, -15, 15)

    elif region_data[0, 3] >= 30:
        steer = np.clip(region_data[0, 2] * 180 / np.pi, -15, 15)

    # elif region_data[0, 3] <= 20:
    #
    #     Rover.mode = 'turn'
    #
    #     # Left or right?
    #     if (region_data[1, 3] + region_data[2, 3]) >= (region_data[3, 3] + region_data[4, 3]):
    #         # steer to left hand side
    #         if region_data[1, 3] >= region_data[2, 3]:
    #             turn_angle = (region_data[1, 2] * 180 / np.pi)
    #         else:
    #             turn_angle = (region_data[2, 2] * 180 / np.pi)
    #     else:
    #         # steer to right hand side
    #         if region_data[3, 3] >= region_data[4, 3]:
    #             turn_angle = (region_data[3, 2] * 180 / np.pi)
    #         else:
    #             turn_angle = (region_data[4, 2] * 180 / np.pi)
    #
    #     Rover.turn_desired_yaw = Rover.yaw + turn_angle
    #
    #     steer = np.clip(turn_angle, -15, 15)

    else:
        # Left or right?
        if (region_data[1, 3] + region_data[2, 3]) >= (region_data[3, 3] + region_data[4, 3]):
            # steer to left hand side
            if region_data[1, 3] >= region_data[2, 3]:
                steer = np.clip(region_data[1, 2] * 180 / np.pi, -15, 15)
            else:
                steer = np.clip(region_data[2, 2] * 180 / np.pi, -15, 15)
        else:
            # steer to right hand side
            if region_data[3, 3] >= region_data[4, 3]:
                steer = np.clip(region_data[3, 2] * 180 / np.pi, -15, 15)
            else:
                steer = np.clip(region_data[4, 2] * 180 / np.pi, -15, 15)


    # #### Draw text
    # #
    cv2.putText(Rover.vision_image, 'M0 mean dist: ' + str(region_data[0, 3]), (2, 48),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(Rover.vision_image, 'L1 mean dist: ' + str(region_data[1, 3]), (2, 64),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(Rover.vision_image, 'L2 mean dist: ' + str(region_data[2, 3]), (2, 80),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(Rover.vision_image, 'R1 mean dist: ' + str(region_data[3, 3]), (2, 96),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(Rover.vision_image, 'R2 mean dist: ' + str(region_data[3, 3]), (2, 112),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)

    #### Draw direction
    #
    for i in range(0, region_data.shape[0]):
        # x_arrow = np.int_(region_M0_mean_dist * np.cos(-region_M0_mean_angle - np.pi / 2)) + 160
        # y_arrow = np.int_(region_M0_mean_dist * np.sin(-region_M0_mean_angle - np.pi / 2)) + 160
        x_arrow = np.int_(region_data[i, 3] * np.cos(-region_data[i, 2] - np.pi / 2)) + 160
        y_arrow = np.int_(region_data[i, 3] * np.sin(-region_data[i, 2] - np.pi / 2)) + 160
        #
        arrow_color = (255, 255, 255)
        cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)

    return steer

def compute_region_data(Rover, region):

    region_M0 = region & (Rover.nav_angles <= (np.pi * 15 / 180)) & (Rover.nav_angles > -(np.pi * 15 / 180))
    region_L1 = region & (Rover.nav_angles <= (np.pi * 45 / 180)) & (Rover.nav_angles > (np.pi * 15 / 180))
    region_L2 = region & (Rover.nav_angles > (np.pi * 45 / 180))
    region_R1 = region & (Rover.nav_angles <= -(np.pi * 15 / 180)) & (Rover.nav_angles > -(np.pi * 45 / 180))
    region_R2 = region & (Rover.nav_angles <= -(np.pi * 45 / 180))

    region_data = np.zeros((5, 4), dtype=np.float)

    # NOTE:
    #
    # region_data[i, 0] : index
    # region_data[i, 1] : pixel count
    # region_data[i, 2] : mean of angle
    # region_data[i, 3] : mean of distance
    #
    # index: {0, 1, 2, 3, 4}
    # 0 : angle between 15 to -15 degree
    # 1 : L1 (Left 1), angle between 15 to 45 degree
    # 2 : L2 (Left 2), angle > 45 degree
    # 3 : R1 (Right 1), angle between -15 to -45 degree
    # 4 : R2 (Right 2), angle < -45 degree

    region_data[0, 0] = 0
    region_data[0, 1] = np.sum(region_M0)
    if region_data[0, 1] > 0:
        region_data[0, 2] = np.mean(Rover.nav_angles[region_M0])
        region_data[0, 3] = np.mean(Rover.nav_dists[region_M0])

    region_data[1, 0] = 0
    region_data[1, 1] = np.sum(region_L1)
    if region_data[1, 1] > 0:
        region_data[1, 2] = np.mean(Rover.nav_angles[region_L1])
        region_data[1, 3] = np.mean(Rover.nav_dists[region_L1])

    region_data[2, 0] = 0
    region_data[2, 1] = np.sum(region_L2)
    if region_data[2, 1] > 0:
        region_data[2, 2] = np.mean(Rover.nav_angles[region_L2])
        region_data[2, 3] = np.mean(Rover.nav_dists[region_L2])

    region_data[3, 0] = 0
    region_data[3, 1] = np.sum(region_R1)
    if region_data[3, 1] > 0:
        region_data[3, 2] = np.mean(Rover.nav_angles[region_R1])
        region_data[3, 3] = np.mean(Rover.nav_dists[region_R1])

    region_data[4, 0] = 0
    region_data[4, 1] = np.sum(region_R2)
    if region_data[4, 1] > 0:
        region_data[4, 2] = np.mean(Rover.nav_angles[region_R2])
        region_data[4, 3] = np.mean(Rover.nav_dists[region_R2])

    return region_M0, region_L1, region_L2, region_R1, region_R2, region_data


# def compute_steer_with_unvisited_2(Rover):
#
#     unvisited = (Rover.nav_dists <= 90) & (Rover.nav_visit_time == 1)
#     #
#     region_M0 = unvisited & (Rover.nav_angles <= (np.pi * 15 / 180)) & (Rover.nav_angles > -(np.pi * 15 / 180))
#     region_L1 = unvisited & (Rover.nav_angles <= (np.pi * 45 / 180)) & (Rover.nav_angles > (np.pi * 15 / 180))
#     region_L2 = unvisited & (Rover.nav_angles > (np.pi * 45 / 180))
#     region_R1 = unvisited & (Rover.nav_angles <= -(np.pi * 15 / 180)) & (Rover.nav_angles > -(np.pi * 45 / 180))
#     region_R2 = unvisited & (Rover.nav_angles <= -(np.pi * 45 / 180))
#
#     region_M0_count = np.sum(region_M0)
#     if region_M0_count > 0:
#         region_M0_mean_angle = np.mean(Rover.nav_angles[region_M0])
#         region_M0_mean_dist  = np.mean(Rover.nav_dists[region_M0])
#     else:
#         region_M0_mean_angle = 0
#         region_M0_mean_dist  = 0
#
#     region_L1_count = np.sum(region_L1)
#     if region_L1_count > 0:
#         region_L1_mean_angle = np.mean(Rover.nav_angles[region_L1])
#         region_L1_mean_dist  = np.mean(Rover.nav_dists[region_L1])
#     else:
#         region_L1_mean_angle = 0
#         region_L1_mean_dist  = 0
#
#     region_L2_count = np.sum(region_L2)
#     if region_L2_count > 0:
#         region_L2_mean_angle = np.mean(Rover.nav_angles[region_L2])
#         region_L2_mean_dist = np.mean(Rover.nav_dists[region_L2])
#     else:
#         region_L2_mean_angle = 0
#         region_L2_mean_dist = 0
#
#     region_R1_count = np.sum(region_R1)
#     if region_R1_count > 0:
#         region_R1_mean_angle = np.mean(Rover.nav_angles[region_R1])
#         region_R1_mean_dist = np.mean(Rover.nav_dists[region_R1])
#     else:
#         region_R1_mean_angle = 0
#         region_R1_mean_dist = 0
#
#     region_R2_count = np.sum(region_R2)
#     if region_R2_count > 0:
#         region_R2_mean_angle = np.mean(Rover.nav_angles[region_R2])
#         region_R2_mean_dist = np.mean(Rover.nav_dists[region_R2])
#     else:
#         region_R2_mean_angle = 0
#         region_R2_mean_dist = 0
#
#
#     #### Draw direction
#     #
#     x_arrow = np.int_(region_M0_mean_dist * np.cos(-region_M0_mean_angle - np.pi / 2)) + 160
#     y_arrow = np.int_(region_M0_mean_dist * np.sin(-region_M0_mean_angle - np.pi / 2)) + 160
#     #
#     arrow_color = (255, 255, 255)
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     x_arrow = np.int_(region_L1_mean_dist * np.cos(-region_L1_mean_angle - np.pi / 2)) + 160
#     y_arrow = np.int_(region_L1_mean_dist * np.sin(-region_L1_mean_angle - np.pi / 2)) + 160
#     #
#     arrow_color = (255, 255, 255)
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     x_arrow = np.int_(region_L2_mean_dist * np.cos(-region_L2_mean_angle - np.pi / 2)) + 160
#     y_arrow = np.int_(region_L2_mean_dist * np.sin(-region_L2_mean_angle - np.pi / 2)) + 160
#     #
#     arrow_color = (255, 255, 255)
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     x_arrow = np.int_(region_R1_mean_dist * np.cos(-region_R1_mean_angle - np.pi / 2)) + 160
#     y_arrow = np.int_(region_R1_mean_dist * np.sin(-region_R1_mean_angle - np.pi / 2)) + 160
#     #
#     arrow_color = (255, 255, 255)
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     x_arrow = np.int_(region_R2_mean_dist * np.cos(-region_R2_mean_angle - np.pi / 2)) + 160
#     y_arrow = np.int_(region_R2_mean_dist * np.sin(-region_R2_mean_angle - np.pi / 2)) + 160
#     #
#     arrow_color = (255, 255, 255)
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#
#     steer = np.clip(region_M0_mean_angle * 180 / np.pi, -15, 15)
#
#     return steer
#
# def compute_steer_with_unvisited_region(Rover):
#
#     steer = 0
#
#     all_region = (Rover.nav_dists <= 90) & (Rover.nav_visit_time == 1)
#     #
#     all_mean_dir = 0
#     all_mean_dist = 0
#     #
#     if np.sum(all_region) > 0:
#         # all_mean_dir = np.mean(Rover.nav_angles[all_region])
#         all_mean_dir = np.average(Rover.nav_angles[all_region], weights=Rover.nav_dists[all_region]**2)
#         # all_mean_dir = np.average(Rover.nav_angles[all_region], weights=Rover.nav_dists[all_region])
#         all_mean_dist = np.mean(Rover.nav_dists[all_region])
#         #
#         # x_arrow = np.int_(all_mean_dist * np.cos(-all_mean_dir - np.pi / 2)) + 160
#         # y_arrow = np.int_(all_mean_dist * np.sin(-all_mean_dir - np.pi / 2)) + 160
#         # #
#         # cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), (255, 255, 255), 4)
#
#     #### Compute steering angle with unvisited region
#     #
#     # the region will be seperated into left & right
#     # left_region = (Rover.nav_dists <= 90) & (Rover.nav_visit_time == 1) & (Rover.nav_angles >= 0)
#     left_region = all_region & (Rover.nav_angles >= 0)
#     #
#     left_mean_dir = 0
#     left_mean_dist = 0
#     #
#     if np.sum(left_region) > 0:
#         left_mean_dir = np.mean(Rover.nav_angles[left_region])
#         # left_mean_dir = np.average(Rover.nav_angles[left_region], weights=Rover.nav_dists[left_region]**2)
#         # left_mean_dir = np.average(Rover.nav_angles[left_region], weights=Rover.nav_dists[left_region])
#         left_mean_dist = np.mean(Rover.nav_dists[left_region])
#         #
#         # x_arrow = np.int_(left_mean_dist * np.cos(-left_mean_dir-np.pi/2)) + 160
#         # y_arrow = np.int_(left_mean_dist * np.sin(-left_mean_dir-np.pi/2)) + 160
#         # #
#         # cv2.arrowedLine(Rover.vision_image, (160,160), (x_arrow, y_arrow), (255,255,255), 4)
#
#     right_region = all_region & (Rover.nav_angles < 0)
#     #
#     right_mean_dir = 0
#     right_mean_dist = 0
#     #
#     if np.sum(left_region) > 0:
#         right_mean_dir = np.mean(Rover.nav_angles[right_region])
#         # right_mean_dir = np.average(Rover.nav_angles[right_region], weights=Rover.nav_dists[right_region]**2)
#         # right_mean_dir = np.average(Rover.nav_angles[right_region], weights=Rover.nav_dists[right_region])
#         right_mean_dist = np.mean(Rover.nav_dists[right_region])
#         #
#         # x_arrow = np.int_(right_mean_dist * np.cos(-right_mean_dir-np.pi/2)) + 160
#         # y_arrow = np.int_(right_mean_dist * np.sin(-right_mean_dir-np.pi/2)) + 160
#         # #
#         # cv2.arrowedLine(Rover.vision_image, (160,160), (x_arrow, y_arrow), (255,255,255), 4)
#
#     #### check the effectiveness of the three directions
#     #
#     all_dir_count = np.sum((Rover.nav_dists <= 90) & (Rover.nav_angles <= all_mean_dir + (np.pi * 10/180)) & (Rover.nav_angles >= all_mean_dir - (np.pi * 10/180)))
#     left_dir_count = np.sum((Rover.nav_dists <= 90) & (Rover.nav_angles <= left_mean_dir + (np.pi * 10/180)) & (Rover.nav_angles >= left_mean_dir - (np.pi * 10/180)))
#     right_dir_count = np.sum((Rover.nav_dists <= 90) & (Rover.nav_angles <= right_mean_dir + (np.pi * 10/180)) & (Rover.nav_angles >= right_mean_dir - (np.pi * 10/180)))
#
#     if len(Rover.nav_dists) > 0:
#         cv2.putText(Rover.vision_image, 'all dir: ' + str(all_dir_count), (2, 64),
#                     cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
#         cv2.putText(Rover.vision_image, 'left dir: ' + str(left_dir_count), (2, 80),
#                     cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
#         cv2.putText(Rover.vision_image, 'right dir: ' + str(right_dir_count), (2, 96),
#                     cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)
#
#     #### Selection criteria by vector length
#     # if all_dir_count >= (left_dir_count + right_dir_count) / 4:
#     #     steer = np.clip(all_mean_dir * 180 / np.pi, -15, 15)
#     #     selected_dir = 'a'
#     # else:
#     #     if left_mean_dist > right_mean_dist:
#     #         steer = np.clip(left_mean_dir * 180 / np.pi, -15, 15)
#     #         selected_dir = 'l'
#     #     else:
#     #         steer = np.clip(right_mean_dir * 180 / np.pi, -15, 15)
#     #         selected_dir = 'r'
#
#     #### Selection criteria by the effectiveness of the direction
#     # if all_dir_count >= left_dir_count and  all_dir_count >= right_dir_count:
#         steer = np.clip(all_mean_dir * 180 / np.pi, -15, 15)
#         selected_dir = 'a'
#     # else:
#     #     if left_mean_dist >= right_mean_dist:
#     #         steer = np.clip(left_mean_dir * 180 / np.pi, -15, 15)
#     #         selected_dir = 'l'
#     #     else:
#     #         steer = np.clip(right_mean_dir * 180 / np.pi, -15, 15)
#     #         selected_dir = 'r'
#
#     #### Draw direction
#     #
#     # all
#     #
#     x_arrow = np.int_(all_mean_dist * np.cos(-all_mean_dir - np.pi / 2)) + 160
#     y_arrow = np.int_(all_mean_dist * np.sin(-all_mean_dir - np.pi / 2)) + 160
#     #
#     if selected_dir == 'a':
#         arrow_color = (0, 0, 255)
#     else:
#         arrow_color = (255, 255, 255)
#     #
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     # left
#     #
#     x_arrow = np.int_(left_mean_dist * np.cos(-left_mean_dir - np.pi / 2)) + 160
#     y_arrow = np.int_(left_mean_dist * np.sin(-left_mean_dir - np.pi / 2)) + 160
#     #
#     if selected_dir == 'l':
#         arrow_color = (0, 0, 255)
#     else:
#         arrow_color = (255, 255, 255)
#     #
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#     #
#     # right
#     #
#     if selected_dir == 'r':
#         arrow_color = (0, 0, 255)
#     else:
#         arrow_color = (255, 255, 255)
#     #
#     x_arrow = np.int_(right_mean_dist * np.cos(-right_mean_dir - np.pi / 2)) + 160
#     y_arrow = np.int_(right_mean_dist * np.sin(-right_mean_dir - np.pi / 2)) + 160
#     #
#     cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)
#
#     return steer

def compute_steer_with_visited_region(Rover):

    # steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
    #
    # # angles_filters = Rover.nav_dists < 60
    # # steer_weighted = np.clip(np.average(Rover.nav_angles[angles_filters] * 180 / np.pi, \
    # #                                     weights=1 / Rover.nav_visited[angles_filters]), -15, 15)
    #
    # steer_weighted = np.clip(np.average(Rover.nav_angles * 180 / np.pi, \
    #                                     weights=1 / Rover.nav_visited), -15, 15)
    #
    # return steer*0.3 + steer_weighted*0.7

    steer = 0

    region = (Rover.nav_dists <= 40) & (Rover.nav_visit_time != 1)

    if np.sum(region) > 0:
        visit_time = np.unique(Rover.nav_visit_time[region])

        if len(visit_time) >= 2:
            visit_time_min = np.min(visit_time)
            visit_time_max = np.min(visit_time)
            #
            region_min = region & (Rover.nav_visit_time == visit_time_min)
            mean_dir_min = np.mean(Rover.nav_angles[region_min])
            mean_dist_min = np.mean(Rover.nav_dists[region_min])

            region_max = region & (Rover.nav_visit_time == visit_time_max)
            mean_dir_max = np.mean(Rover.nav_angles[region_max])
            mean_dist_max = np.mean(Rover.nav_dists[region_max])

            #### Draw direction
            #
            # visit_time_min
            #
            x_arrow = np.int_(mean_dist_min * np.cos(-mean_dir_min - np.pi / 2)) + 160
            y_arrow = np.int_(mean_dist_min * np.sin(-mean_dir_min - np.pi / 2)) + 160
            #
            arrow_color = (255, 191, 0)
            #
            cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)

            # visit_time_max
            #
            x_arrow = np.int_(mean_dist_max * np.cos(-mean_dir_max - np.pi / 2)) + 160
            y_arrow = np.int_(mean_dist_max * np.sin(-mean_dir_max - np.pi / 2)) + 160
            #
            arrow_color = (255, 128, 0)
            #
            cv2.arrowedLine(Rover.vision_image, (160, 160), (x_arrow, y_arrow), arrow_color, 4)

            steer = np.clip(mean_dir_min * 180 / np.pi, -15, 15)

    return steer

def pickup(Rover):

    if Rover.picking_up:
        # Do nothing
        Rover.throttle = 0

    elif Rover.near_sample:
        if not Rover.picking_up:
            if Rover.vel == 0:
                Rover.brake = 0
                Rover.send_pickup = True
                #
                # Rover.throttle_set += 0.2
            else:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
    else:
        # most likely the pickup action was done
        Rover.mode = 'forward'

    return

def drive_to_rock(Rover):

    if is_rock_found(Rover):
        Rover.throttle = 0.05
        # Set steering to average angle clipped to the range +/- 15
        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)

    else:
        # back to forward forward mode when the rock missed
        Rover.mode = 'forward'

    return

def drive_forward(Rover):

    if Rover.vel == 0 and abs(Rover.steer) >= 4 and Rover.throttle > 0:
        # Escape from stuck
        Rover.message = 'get stuck'
        Rover.throttle = 0
        Rover.brake = 0
        return

    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        if Rover.vel < Rover.max_vel:
            # Set throttle value to throttle setting
            Rover.throttle = Rover.throttle_set
        else:  # Else coast
            Rover.throttle = 0
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        # angles_filters = Rover.nav_dists < 60
        # Rover.steer = np.clip(np.average(Rover.nav_angles[angles_filters] * 180 / np.pi, \
        #                                weights=1/Rover.nav_visited[angles_filters]), -15, 15)
        Rover.steer = compute_steer(Rover)

    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(Rover.nav_angles) < Rover.stop_forward*2:
        # Set mode to "stop" and hit the brakes!
        Rover.throttle = 0
        # Set brake to stored brake value
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

    return

def drive_stop(Rover):

    # If we're in stop mode but still moving keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
    # If we're not moving (vel < 0.2) then do something else
    elif Rover.vel <= 0.2:
        # Now we're stopped and we have vision data to see if there's a path forward
        if len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            if Rover.steer > 0:
                Rover.steer = 15  # Could be more clever here about which way to turn
            else:
                Rover.steer = -15
                # Rover.steer = -15  # Could be more clever here about which way to turn
        # If we're stopped but see sufficient navigable terrain in front then go!
        if len(Rover.nav_angles) >= Rover.go_forward * 4:
            # Set throttle back to stored value
            Rover.throttle = Rover.throttle_set
            # Release the brake
            Rover.brake = 0
            # Set steer to mean angle
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            Rover.mode = 'forward'

    return

def drive_turn(Rover):

    # #### Draw text
    # #
    cv2.putText(Rover.vision_image, 'Desired yaw: ' + str(Rover.turn_desired_yaw), (2, 48),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)

    cv2.putText(Rover.vision_image, 'Yaw error: ' + str(Rover.turn_desired_yaw - Rover.yaw), (2, 64),
                cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255), 1)

    Rover.throttle = 0

    Rover.steer = np.clip(Rover.turn_desired_yaw - Rover.yaw, -15, 15)

    if abs(Rover.turn_desired_yaw - Rover.yaw) <= 5:
        Rover.mode = 'forward'

    return Rover
