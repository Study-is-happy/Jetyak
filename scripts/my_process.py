import numpy as np
import cv2
import time
import geopandas
import multiprocessing
import rospy

from shapely.geometry import Polygon, LineString, Point, MultiPoint
from shapely import affinity
from scipy.stats import multivariate_normal
from skimage import draw

from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointPull
from mavros_msgs.msg import WaypointList, Waypoint

import my_filter
# import my_plot
# import my_satellite_plot
import my_util
import my_cpa

input_waypoints = []
with open("/home/zhiyongzhang/dataset/HendersonToMIT.waypoints") as waypoints_file:
    waypoints_file.readline()
    for raw_waypoint in waypoints_file:
        waypoint = raw_waypoint.strip().split("\t")
        input_waypoints.append(
            my_util.latlng_to_utm(np.array(waypoint[8:10], np.float32)))


land_area_shp = geopandas.read_file(
    "/home/zhiyongzhang/dataset/shapefiles/BostonUTM.shp")

bridge_hole_shp = geopandas.read_file(
    "/home/zhiyongzhang/dataset/bridgesshapefiles/Bridges.shp")


img_dim = 2000

# plot_img_dim = 640

ship_data = []


def get_path_distance(path):

    path2 = path.copy()

    path = np.append(path, [path[-1]], 0)

    path2 = np.insert(path2, 0, path2[0], 0)

    return np.sum(np.sqrt(np.sum(np.square(path - path2), 1)))


def check_polygon(current_polygons, shapely_contour):

    for current_polygon in current_polygons:
        if shapely_contour.intersects(current_polygon):
            return True
    return False


def check_border(path):
    return np.any((path == 0) | (path == img_dim * 2 - 1))


def find_valid_contour(border):

    return np.invert(np.all([border, np.roll(border, -1, 0), np.roll(border, 1, 0)], 0))


def find_border(img_contour):
    return np.any((img_contour == 0) | (img_contour == img_dim * 2 - 1), -1)


def check_outside(point):
    return np.any((point < 0) | (point >= img_dim * 2))


def check_inside(points):
    return np.any(np.all((points >= 0) & (points < img_dim * 2), -1))


def check_valid_point(process_img, point):
    return not check_outside(point) and process_img[point[0]][point[1]] == 0


def find_valid_point(process_img, point):
    temp_point = np.array([point[0] + 1, point[1]])
    if check_valid_point(process_img, temp_point):
        return temp_point
    temp_point = np.array([point[0] - 1, point[1]])
    if check_valid_point(process_img, temp_point):
        return temp_point
    temp_point = np.array([point[0], point[1] + 1])
    if check_valid_point(process_img, temp_point):
        return temp_point
    temp_point = np.array([point[0], point[1] - 1])
    if check_valid_point(process_img, temp_point):
        return temp_point


def find_nearest_index(points, point):
    return np.argmin(np.sqrt(np.sum(np.square(points - point), -1)))


def find_nearest_contour_indexs(img_contour, point):
    argmins = []
    points = []
    for contour in img_contour:

        argmin = find_nearest_index(contour, point)
        argmins.append(argmin)
        points.append(contour[argmin])

    argmin = find_nearest_index(points, point)

    return [argmin, argmins[argmin]]


def check_path(process_img, point1, point2):

    path_is, path_js = draw.line(point1[0], point1[1],
                                 point2[0], point2[1])

    return not np.any(process_img[path_is[1:-1], path_js[1:-1]] != 0)


def binary_search(line_points):
    for index, line_point in enumerate(line_points):
        if check_outside(line_point):
            return index - 1


def optimizate_path(process_img, current_path):
    start_index = 0

    while start_index < len(current_path) - 2:
        start_point = current_path[start_index]
        end_index = len(current_path) - 1
        while end_index > start_index + 1:
            end_point = current_path[end_index]
            if check_path(process_img, start_point, end_point):
                current_path = np.delete(current_path, xrange(
                    start_index + 1, end_index), 0)
                break
            end_index -= 1
        start_index += 1
    return current_path


def get_cv2_kernel(kernel_distance):

    kernel_n = kernel_distance * 2 + 1

    kernel_x, kernel_y = np.ogrid[-kernel_distance:kernel_n - kernel_distance, -
                                  kernel_distance:kernel_n - kernel_distance]
    kernel_mask = kernel_x ** 2 + kernel_y ** 2 <= kernel_distance**2

    kernel = np.zeros((kernel_n, kernel_n))

    kernel[kernel_mask] = 1

    return np.uint8(kernel)


def remove_self(amplitudes):
    start_decrease = False
    index = 1
    while index < len(amplitudes):

        if amplitudes[index] < amplitudes[index - 1]:
            start_decrease = True

        elif start_decrease:
            amplitudes[index - 1] = np.uint8(0)
            break

        amplitudes[index - 1] = np.uint8(0)
        index += 1


def get_ellipse(center, cov):
    vals, vecs = np.linalg.eigh(cov)
    angle = np.arctan2(vecs[1, 0], vecs[0, 0])
    weight, height = np.sqrt(vals)
    return np.array(affinity.rotate(affinity.scale(Point(center).buffer(
        1.), weight, height), np.degrees(angle)).exterior)


def process(radar_data, config):

    start = time.time()

    radar_data["scanline"].append(radar_data["scanline"][0])
    radar_data["utm"].append(radar_data["utm"][0])
    radar_data["angle"].append(radar_data["angle"][0])

    scanlines = []

    for scanline in radar_data["scanline"]:

        amplitudes = np.array([np.uint8(ord(raw_amplitude))
                               for raw_amplitude in scanline])

        remove_self(amplitudes)

        amplitudes[amplitudes < 100] = 0

        scanlines.append(amplitudes)

    del radar_data["scanline"]

    cv2_contours, _ = cv2.findContours(
        np.array(scanlines), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    del scanlines

    radar_data["utm"] = np.array(radar_data["utm"])
    radar_data["angle"] = np.array(radar_data["angle"])

    self_utm = np.array(config["self_utm"])
    self_angle = np.pi / 2 - config["self_angle"]
    self_speed = config["self_speed"]
    # img_scale = img_dim / config["radar_distance"]
    img_scale = 2.

    process_img = np.zeros(
        (img_dim * 2, img_dim * 2), np.uint8)

    # plot_img = np.empty(
    #     (plot_img_dim * 2, plot_img_dim * 2, 3), np.uint8)
    # plot_img.fill(255)

    # plot_img_scale = 2. / (np.cos(np.radians(my_util.utm_to_latlng(self_utm)[0])) * 2 *
    #                        np.pi * 6378137 / (256. * 2.**17))

    for cv2_contour in cv2_contours:

        cv2_contour = np.squeeze(cv2_contour, -2)

        distances = (cv2_contour[:, 0] + 1.) / 512. * \
            config["radar_distance"] * img_scale

        angles = radar_data["angle"][cv2_contour[:, 1]]

        offsets = my_util.utm_to_img_index(
            radar_data["utm"][cv2_contour[:, 1]], self_utm, img_scale, img_dim)

        points = my_util.get_round_int(np.array([np.sin(angles) * distances,
                                                 np.cos(angles) * distances]).T + offsets)

        cv2.fillPoly(process_img, [points], 1)

    del radar_data

    cv2_contours, _ = cv2.findContours(
        process_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    current_land_polygons = []

    for land_polygon in land_area_shp.geometry:

        points = my_util.get_round_int(my_util.utm_to_img_index(np.array(land_polygon.exterior),
                                                                self_utm, img_scale, img_dim))
        if check_inside(points):

            current_land_polygons.append(Polygon(points))

            buffer_land_polygon = land_polygon.buffer(10.)

            buffer_points = my_util.get_round_int(my_util.utm_to_img_index(np.array(buffer_land_polygon.exterior),
                                                                           self_utm, img_scale, img_dim))

            cv2.fillPoly(process_img, [buffer_points], 1)

            # cv2.fillPoly(plot_img, [my_util.img_to_plot_img(
            #     points, self_utm, img_scale, img_dim, plot_img_scale,
            #     plot_img_dim)], (189, 195, 199))

    current_bridge_polygons = []

    for bridge_polygon in bridge_hole_shp.geometry:

        points = my_util.get_round_int(my_util.utm_to_img_index(np.array(bridge_polygon.exterior),
                                                                self_utm, img_scale, img_dim))

        if check_inside(points):

            current_bridge_polygons.append(Polygon(points))

            cv2.fillPoly(process_img, [points], 0)

            # cv2.fillPoly(plot_img, [my_util.img_to_plot_img(
            #     points, self_utm, img_scale, img_dim, plot_img_scale,
            #     plot_img_dim)], (189, 195, 199))

    temp_ship_data = []

    for cv2_contour in cv2_contours:

        cv2_contour = np.squeeze(cv2_contour, -2)

        if len(cv2_contour) == 1:
            shapely_contour = Point(cv2_contour[0])
        elif len(cv2_contour) == 2:
            shapely_contour = LineString(cv2_contour)
        else:
            shapely_contour = Polygon(cv2_contour)

        if not shapely_contour.is_valid:
            shapely_contour = shapely_contour.buffer(0.1)

        if check_polygon(current_land_polygons, shapely_contour) or check_polygon(current_bridge_polygons, shapely_contour):

            pass
            # cv2.polylines(plot_img, [my_util.img_to_plot_img(
            #     cv2_contour, self_utm, img_scale, img_dim, plot_img_scale,
            #     plot_img_dim)], True, (220, 220, 220), 4)

        else:

            rect = list(cv2.minAreaRect(
                np.float32(my_util.img_index_to_utm(cv2_contour, self_utm, img_scale, img_dim))))

            rect[1] = list(rect[1])

            if rect[1][0] < rect[1][1]:
                rect[2] += 90
                rect[1][0], rect[1][1] = rect[1][1], rect[1][0]

            temp_ship_data.append({"measurement":
                                   [rect[0][0], rect[0][1], np.deg2rad(rect[2])], "rect": rect})

            # cv2.fillPoly(plot_img, [my_util.img_to_plot_img(
            #     cv2_contour, self_utm, img_scale, img_dim, plot_img_scale,
            #     plot_img_dim)], (255, 0, 0))

    del cv2_contours

    del current_land_polygons

    del current_bridge_polygons

    # data association

    global associations
    global associations_pdf

    associations = []
    associations_pdf = -1

    global ship_data

    def get_associations(temp_associations, temp_associations_pdf):
        temp_ship_index = len(temp_associations)
        if temp_ship_index == len(temp_ship_data):
            global associations
            global associations_pdf
            if temp_associations_pdf > associations_pdf:
                associations = temp_associations
                associations_pdf = temp_associations_pdf
        else:
            temp_ship = temp_ship_data[temp_ship_index]
            for index, ship in enumerate(ship_data):
                if index not in temp_associations:
                    pdf = ship["pdf"].pdf(
                        temp_ship["measurement"][0:2])

                    distance = my_util.get_distance(
                        temp_ship["rect"][0], ship["rect"][0])

                    if pdf > 0.000001 and distance < config["radar_dt"] * 10:
                        get_associations(
                            temp_associations + [index], temp_associations_pdf + pdf)

            get_associations(
                temp_associations + [-1], temp_associations_pdf)

    get_associations([], 0)

    current_ship_data = []

    for index, association in enumerate(associations):
        temp_ship = temp_ship_data[index]

        if association == -1:
            CTRV_x = np.array(
                [temp_ship["measurement"][0], temp_ship["measurement"][1], 0., temp_ship["measurement"][2], 0.])
            CTRV_P = np.diag(
                [1.**2, 1.**2, 5.**2, (np.pi / 8)**2, (np.pi / 12)**2])
            CTRV_R = np.diag([1.**2, 1.**2, (np.pi / 8)**2])
            CTRV_Q = np.diag(
                [(0.5 * 0.5 * config["radar_dt"]**2)**2,
                 (0.5 * 0.5 * config["radar_dt"]**2)**2,
                 (0.5 * config["radar_dt"])**2,
                 (np.pi / 48 * config["radar_dt"])**2,
                 (np.pi / 48 * config["radar_dt"])**2])
            CTRV_Q_aug = np.diag([0.5**2, (np.pi / 48)**2])

            ship = {
                # "plot_trajectory": [],
                "CTRV": my_filter.CTRV(
                    CTRV_x, CTRV_P, CTRV_R, CTRV_Q, CTRV_Q_aug, config["radar_dt"])}

        else:
            ship = ship_data[association]

            angle_diff = my_util.normalize_radian(
                temp_ship["measurement"][2] - np.deg2rad(ship["rect"][2]))

            if abs(angle_diff) > np.pi / 2:
                temp_ship["measurement"][2] += np.pi
                temp_ship["rect"][2] += 180

            ship["CTRV"].update(np.array(temp_ship["measurement"]))

            ship_center = tuple(ship["CTRV"].x[0:2])

            ship_speed = ship["CTRV"].x[2]

            ship_speed_dev = np.sqrt(ship["CTRV"].P[2][2])

            ship_angle = ship["CTRV"].x[3]

            ship_angle_dev = np.sqrt(ship["CTRV"].P[3][3])

            if ship_speed < 0:
                ship_speed = -ship_speed
                ship_angle += np.pi

            ellipse_points = get_ellipse(ship_center, ship["CTRV"].P[0:2, 0:2])

            ship_points = []

            for ellipse_point in ellipse_points:
                ship_points.extend(cv2.boxPoints((ellipse_point,
                                                  temp_ship["rect"][1], np.degrees(ship_angle))))

            del ellipse_points

            ship_points = np.squeeze(
                cv2.convexHull(np.array(ship_points)))

            long_axis = Point(ship_center).hausdorff_distance(
                Polygon(ship_points))

            rotate_angles = np.linspace(-ship_angle_dev, ship_angle_dev, 10)

            projection_points = []

            projection_distance = (ship_speed + ship_speed_dev) * 5

            for rotate_angle in rotate_angles:

                rotate_ship_points = np.array(affinity.rotate(MultiPoint(ship_points),
                                                              rotate_angle, ship_center, use_radians=True))

                projection_points.extend(rotate_ship_points)

                relative_angle = rotate_angle + ship_angle

                for rotate_ship_point in rotate_ship_points:

                    projection_points.append(my_util.get_rotate_point(
                        rotate_ship_point, relative_angle, projection_distance))

            del rotate_angles

            projection_points = cv2.convexHull(
                np.array(projection_points, np.float32))

            projection_points = my_util.get_round_int(my_util.utm_to_img_index(
                projection_points, self_utm, img_scale, img_dim))

            cv2.fillConvexPoly(
                process_img, projection_points, 1)

            # cv2.polylines(
            #     plot_img, [my_util.img_to_plot_img(
            #         projection_points, self_utm, img_scale,
            #         img_dim, plot_img_scale, plot_img_dim)], True, (255, 0, 0), 5)

            del projection_points

            relative_angle = np.arctan2(
                ship_center[1] - self_utm[1], ship_center[0] - self_utm[0])

            if abs(my_util.normalize_radian(relative_angle - self_angle)) < np.pi / 2:

                cpa = my_cpa.CPA(self_utm, ship_center, self_speed,
                                 ship_speed, self_angle, ship_angle)

                if cpa.get_t() >= 0 and cpa.get_t() <= 60 and cpa.get_d() <= 50 + long_axis + config["self_bevel"]:

                    angle_diff = np.degrees(
                        my_util.normalize_radian(self_angle - ship_angle))

                    if angle_diff < 0:
                        angle_diff += 360

                    COLREGS_projection_distance = projection_distance + long_axis

                    COLREGS_points = [ship_center]

                    if angle_diff >= 165 and angle_diff <= 195:
                        rotate_angles = np.linspace(-np.pi / 2,
                                                    0, 10) + ship_angle

                        for rotate_angle in rotate_angles:
                            COLREGS_points.append(my_util.get_rotate_point(
                                ship_center, rotate_angle, COLREGS_projection_distance * 2))

                    elif angle_diff > 195 and angle_diff < 292.5:
                        rotate_angles = np.linspace(-ship_angle_dev,
                                                    ship_angle_dev, 10) + ship_angle

                        for rotate_angle in rotate_angles:
                            COLREGS_points.append(my_util.get_rotate_point(
                                ship_center, rotate_angle, COLREGS_projection_distance * 2))

                    elif angle_diff >= 292.5 or angle_diff <= 67.5:
                        rotate_angles = np.linspace(-np.pi / 4,
                                                    np.pi / 4, 10) + ship_angle

                        for rotate_angle in rotate_angles:
                            COLREGS_points.append(my_util.get_rotate_point(
                                ship_center, rotate_angle, COLREGS_projection_distance))

                    else:
                        pass

                    COLREGS_points = cv2.convexHull(
                        np.array(COLREGS_points, np.float32))

                    COLREGS_points = my_util.get_round_int(my_util.utm_to_img_index(
                        COLREGS_points, self_utm, img_scale, img_dim))

                    cv2.fillConvexPoly(
                        process_img, COLREGS_points, 1)

                    # cv2.polylines(
                    #     plot_img, [my_util.img_to_plot_img(
                    #         COLREGS_points, self_utm, img_scale,
                    #         img_dim, plot_img_scale, plot_img_dim)], True, (0, 0,
                    #                                                         255), 5)

        ship["CTRV"].predict()

        ship["pdf"] = multivariate_normal(
            mean=ship["CTRV"].z_pred[0:2], cov=ship["CTRV"].P[0:2, 0:2])

        ship["rect"] = tuple(temp_ship["rect"])

        # ship["plot_trajectory"].append(temp_ship["measurement"])

        # if len(ship["plot_trajectory"]) > 1:
        #     utm_plot_trajectory = np.array(ship["plot_trajectory"])[:, 0:2]
        #     img_index_plot_trajectory = my_util.get_round_int(my_util.utm_to_img_index(
        #         utm_plot_trajectory, self_utm, img_scale, img_dim))
        #     cv2.polylines(
        #         plot_img, [my_util.img_to_plot_img(
        #             img_index_plot_trajectory, self_utm, img_scale, img_dim,
        #             plot_img_scale, plot_img_dim)], False, (255, 0, 0), 4)

        current_ship_data.append(ship)

    ship_data = current_ship_data

    # shortest path

    safe_distance = config["self_length"] * img_scale * 1.5

    process_img = cv2.dilate(
        process_img, get_cv2_kernel(int(safe_distance * 1.2)))

    label_count, label_img = cv2.connectedComponents(process_img)

    img_contours, _ = cv2.findContours(
        process_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    img_contour_dict = {}
    for i in xrange(1, label_count):
        img_contour_dict[i] = []

    img_border_dict = {}

    for img_contour in img_contours:

        img_contour = np.flip(np.squeeze(img_contour, -2), -1)

        first_point = img_contour[0]
        label = label_img[first_point[0]][first_point[1]]

        border = find_border(img_contour)
        valid_contour = find_valid_contour(border)

        img_contour_dict[label].append(img_contour[valid_contour])

        img_border_dict[label] = img_contour[
            np.all([border, valid_contour], 0)]

    del img_contours

    self_point = np.flip(my_util.get_round_int(my_util.utm_to_img_index(
        np.array(self_utm), self_utm, img_scale, img_dim)))

    path = [self_point]

    self_label = label_img[self_point[0]][self_point[1]]

    if self_label != 0:
        img_contour = img_contour_dict[self_label]

        nearest_indexs = find_nearest_contour_indexs(img_contour, self_point)
        nearest_point = img_contour[nearest_indexs[0]][nearest_indexs[1]]
        valid_point = find_valid_point(process_img, nearest_point)
        path.append(valid_point)

    while len(input_waypoints) != 0:

        first_waypoint = np.flip(my_util.get_round_int(my_util.utm_to_img_index(
            input_waypoints[0], self_utm, img_scale, img_dim)))

        if my_util.get_distance(first_waypoint, self_point) < 10.:
            del input_waypoints[0]
        else:
            break

    if len(input_waypoints) != 0:

        line_points = np.array(draw.line(path[-1][0], path[-1][1],
                                         first_waypoint[0], first_waypoint[1])).T

        if check_outside(first_waypoint):
            border_index = binary_search(line_points)
            line_point = line_points[border_index]
            label = label_img[line_point[0]][line_point[1]]
            if label == 0:
                line_points = line_points[:border_index + 1]
                path.append(line_point)
            else:
                img_border = img_border_dict[label]

                border_point = img_border[
                    find_nearest_index(img_border, line_point)]
                valid_point = find_valid_point(process_img, border_point)

                line_points = np.array(draw.line(path[-1][0], path[-1][1],
                                                 valid_point[0], valid_point[1])).T

                path.append(valid_point)

        else:
            label = label_img[first_waypoint[0]][first_waypoint[1]]
            if label == 0:
                path.append(first_waypoint)
            else:
                img_contour = img_contour_dict[label]
                nearest_indexs = find_nearest_contour_indexs(
                    img_contour, first_waypoint)
                nearest_point = img_contour[
                    nearest_indexs[0]][nearest_indexs[1]]

                valid_point = find_valid_point(process_img, nearest_point)

                line_points = np.array(draw.line(path[-1][0], path[-1][1],
                                                 valid_point[0], valid_point[1])).T

                path.append(valid_point)

        previous_label = 0
        previous_point = None
        first_label = 0

        for line_point in line_points[1:]:

            label = label_img[line_point[0]][line_point[1]]

            # start or end+1
            if label != previous_label:

                # start
                if first_label == 0:
                    first_label = label
                    img_contour = img_contour_dict[label]
                    first_indexs = find_nearest_contour_indexs(
                        img_contour, line_point)

                # end+1
                elif previous_label == first_label:
                    img_contour = img_contour_dict[previous_label]
                    last_indexs = find_nearest_contour_indexs(
                        img_contour, previous_point)

                    # same dim
                    if first_indexs[0] == last_indexs[0]:

                        first_index = first_indexs[1]
                        last_index = last_indexs[1]
                        img_contour = img_contour[first_indexs[0]]

                        img_contour_len = len(img_contour)

                        double_img_contour = np.concatenate(
                            [img_contour, img_contour])

                        if first_index < last_index:
                            left_path = double_img_contour[
                                first_index + img_contour_len:last_index - 1:-1]

                            right_path = img_contour[
                                first_index:last_index + 1]

                        else:
                            left_path = double_img_contour[
                                first_index + img_contour_len:last_index + img_contour_len - 1:-1]

                            right_path = double_img_contour[
                                first_index:last_index + img_contour_len + 1]

                        left_invalid = check_border(left_path[:-1])

                        right_invalid = check_border(right_path[:-1])

                        if left_invalid and right_invalid:
                            break

                        elif left_invalid:
                            current_path = right_path
                        elif right_invalid:
                            current_path = left_path
                        else:
                            if get_path_distance(left_path) < get_path_distance(right_path):
                                current_path = left_path
                            else:
                                current_path = right_path

                        path = np.insert(path, -1, current_path, 0)

                        first_label = 0

            previous_point = line_point
            previous_label = label

        if first_label != 0:

            path[-1] = img_contour_dict[first_label][first_indexs[0]][first_indexs[1]]

    del img_border_dict

    del label_img

    del img_contour_dict

    if len(path) != 1:

        path = np.array(path)

        process_img = cv2.erode(
            process_img, get_cv2_kernel(int(safe_distance * 0.2)))
        path = cv2.approxPolyDP(path, img_scale, False)
        path = np.squeeze(path, -2)
        path = optimizate_path(process_img, path)

        # cv2.polylines(plot_img, [my_util.img_to_plot_img(
        #     np.flip(path, -1), self_utm, img_scale, img_dim, plot_img_scale, plot_img_dim)],
        #     False, (247, 220, 111), 4)

        path = path[1:]

        waypointList = WaypointList()

        for utm in my_util.img_index_to_utm(path, self_utm, img_scale, img_dim):
            latlng = my_util.utm_to_latlng(utm)
            waypoint = Waypoint()
            waypoint.frame = 3
            waypoint.command = 16
            waypoint.is_current = False
            waypoint.autocontinue = True
            waypoint.param1 = 0
            waypoint.param2 = 0
            waypoint.param3 = 0
            waypoint.param4 = 0
            waypoint.x_lat = latlng[0]
            waypoint.y_long = latlng[1]
            waypoint.z_alt = 100
            waypointList.waypoints.append(waypoint)

        waypointList.waypoints[0].is_current = True

        waypointPush_service = rospy.ServiceProxy(
            "/mavros/mission/push", WaypointPush)
        waypointPush_service.call(0, waypointList.waypoints)

        del waypointList

        # for point in path:
        #     cv2.circle(plot_img, tuple(my_util.img_to_plot_img(
        #         np.flip(point), self_utm, img_scale, img_dim, plot_img_scale, plot_img_dim)),
        #         8, (241, 196, 15), -1)
    else:
        pass
        waypointClear_service = rospy.ServiceProxy(
            "/mavros/mission/clear", WaypointClear)
        waypointClear_service.call()

    # plot_self_points = my_util.get_round_int(cv2.boxPoints((
    #     my_util.utm_to_img_index(self_utm, self_utm, img_scale, img_dim),
    #     (config["self_length"] * img_scale * 5, config["self_width"] * img_scale
    #      * 5), np.degrees(self_angle))))

    # cv2.fillPoly(plot_img, [my_util.img_to_plot_img(
    #     plot_self_points, self_utm, img_scale, img_dim, plot_img_scale,
    #     plot_img_dim)], (0, 255, 0))

    end = time.time()
    print "Time:", end - start
    print "-----------------------------------------"

    # multiprocessing.Process(target=my_satellite_plot.plot,
    # args=(plot_img, self_utm, config["radar_dt"])).start()

    # multiprocessing.Process(target=my_plot.plot,
    #                         args=(plot_img, config["radar_dt"])).start()
