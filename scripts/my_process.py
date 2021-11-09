import numpy as np
import cv2
import time
import geopandas
import itertools
import shapely
import scipy
import scipy.stats
import os
import matplotlib.pyplot as plt


import Kalman_filter
# import my_plot
import my_util
# import my_cpa
import cache_satellite_images

import rospy
import json
import base64

input_waypoints = []
with open('HendersonToMIT_2.waypoints') as waypoints_file:
    waypoints_file.readline()
    for waypoint in waypoints_file:
        waypoint = np.float64(waypoint.strip().split('\t')[8:10])
        input_waypoints.append(
            my_util.latlng_to_utm(waypoint))

land_polygons = []
for land_polygon in geopandas.read_file('shape_files/lands.shp').geometry:
    land_polygons.append(land_polygon)

bridge_polygons = []
for bridge_polygon in geopandas.read_file('shape_files/bridges.shp').geometry:
    bridge_polygons.append(bridge_polygon)

boundary_polygons = []
for boundary_polygon in geopandas.read_file('shape_files/boundaries.shp').geometry:
    boundary_polygons.append(boundary_polygon)

static_obstacle_polygon = shapely.ops.unary_union(land_polygons + bridge_polygons + boundary_polygons)

hole_polygons = []
for hole_polygon in geopandas.read_file('shape_files/holes.shp').geometry:
    hole_polygons.append(hole_polygon)
hole_polygons = shapely.geometry.MultiPolygon(hole_polygons)


image_utm_ratio = 2.0
half_image_dim = 640

prev_target_list = []
cache_frames = 2

# time_stamp_sec = str(1553700607)

# with open('prev_target_list/' + time_stamp_sec + '.json') as prev_target_list_file:
#     prev_target_list = json.load(prev_target_list_file)
#     prev_target_list = json.loads(prev_target_list)

#     for prev_target in prev_target_list:

#         prev_target['x'] = np.array(prev_target['x'])
#         prev_target['P'] = np.array(prev_target['P'])
#         prev_target['pdf'] = scipy.stats.multivariate_normal(
#             mean=prev_target['x'][:2], cov=prev_target['P'][:2, :2])

# with open('radar_data/' + time_stamp_sec + '.json') as radar_data_file:
#     radar_data = json.load(radar_data_file)
#     radar_data = json.loads(radar_data)
#     for index, scanline in enumerate(radar_data['scanline']):
#         radar_data['scanline'][index] = base64.b64decode(scanline)

# with open('jetyak_config/' + time_stamp_sec + '.json') as jetyak_config_file:
#     jetyak_config = json.load(jetyak_config_file)
#     jetyak_config = json.loads(jetyak_config)
#     jetyak_config['yaw'] = np.array(jetyak_config['yaw'])


class NumpyEncoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, scipy.stats._multivariate.multivariate_normal_frozen):
            return None
        elif isinstance(obj, bytes):
            return base64.b64encode(obj).decode('UTF8')
        return json.JSONEncoder.default(self, obj)


def find_outside_point(point, obstacle_polygons):

    obstacle_lines = []

    obstacle_polygons = obstacle_polygons.buffer(1.)

    if isinstance(obstacle_polygons, shapely.geometry.Polygon):
        obstacle_polygons = [obstacle_polygons]

    for obstacle_polygon in obstacle_polygons:
        obstacle_lines.append(obstacle_polygon.exterior)
        for obstacle_polygon_interior in obstacle_polygon.interiors:
            obstacle_lines.append(obstacle_polygon_interior)

    obstacle_lines = shapely.geometry.MultiLineString(obstacle_lines)

    return shapely.ops.nearest_points(point, obstacle_lines)[1]


def find_nearest_index(points, point):
    return np.argmin(np.sqrt(np.sum(np.square(points - point), -1)))


def process(radar_data, jetyak_config, time_stamp):

    global prev_target_list

    # dumped_prev_target_list = json.dumps(prev_target_list, cls=NumpyEncoder)
    # with open('prev_target_list/' + str(time_stamp.secs) + '.json', 'w') as json_file:
    #     json.dump(dumped_prev_target_list, json_file)

    # dumped_radar_data = json.dumps(radar_data, cls=NumpyEncoder)
    # with open('radar_data/' + str(time_stamp.secs) + '.json', 'w') as json_file:
    #     json.dump(dumped_radar_data, json_file)

    # dumped_jetyak_config = json.dumps(jetyak_config, cls=NumpyEncoder)
    # with open('jetyak_config/' + str(time_stamp.secs) + '.json', 'w') as json_file:
    #     json.dump(dumped_jetyak_config, json_file)

    start = time.time()

    jetyak_utm = radar_data['utm'][-1]
    jetyak_point = shapely.geometry.Point(jetyak_utm)
    jetyak_yaw = np.pi / 2 - radar_data['angle'][-1]
    radar_dt = jetyak_config['radar_dt']

    # plot_image = np.zeros((half_image_dim * 2, half_image_dim * 2, 3), np.uint8)

    image_file_name = 'satellite_images/' + str(jetyak_utm[0]) + '_' + str(jetyak_utm[1]) + '.png'
    if not os.path.exists(image_file_name):
        cache_satellite_images.cache_satellite_images(jetyak_utm[0], jetyak_utm[1])

    plot_image = cv2.imread(image_file_name)
    plot_image = cv2.cvtColor(np.flipud(plot_image), cv2.COLOR_BGR2RGB)

    def fill_polygon_alpha(polygon, color, alpha):

        points = [my_util.utm_to_image(np.array(polygon.exterior), jetyak_utm, image_utm_ratio, half_image_dim)]

        for interior in polygon.interiors:
            points.append(my_util.utm_to_image(np.array(interior), jetyak_utm, image_utm_ratio, half_image_dim))

        layer = plot_image.copy()

        cv2.fillPoly(layer, points, color)

        cv2.addWeighted(plot_image, alpha, layer, 1 - alpha, 0, plot_image)

        cv2.polylines(plot_image, points, True, color, 2)

    # fill_polygon_alpha(static_obstacle_polygon, (189, 195, 199), 0)

    radar_data['scanline'].extend(radar_data['scanline'][:10])
    radar_data['angle'].extend(radar_data['angle'][:10])
    radar_data['utm'].extend(radar_data['utm'][:10])

    radar_data['angle'] = np.array(radar_data['angle'])
    radar_data['utm'] = np.array(radar_data['utm'])

    plot_jetyak_polygon = shapely.geometry.Polygon(cv2.boxPoints((
        jetyak_utm, (jetyak_config['length'] * 5, jetyak_config['width'] * 5), np.degrees(jetyak_yaw))))

    fill_polygon_alpha(plot_jetyak_polygon, (0, 255, 0), 0.4)

    # for radar_utm in radar_data['utm']:
    #     cv2.circle(plot_image, tuple(my_util.utm_to_image(radar_utm, jetyak_utm, image_utm_ratio, half_image_dim)), int(5 * image_utm_ratio), (0, 255, 0), 2)

    scanlines = []

    for scanline in radar_data['scanline']:

        scanline = np.frombuffer(scanline, dtype=np.uint8).copy()

        # scanline[: 13] = 255
        scanline[: np.argmax(scanline == 0)] = 0

        # scanline[scanline < 255] = 0

        scanlines.append(scanline)

    radar_contours, _ = cv2.findContours(
        np.array(scanlines), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    radar_polygons = []

    for radar_contour in radar_contours:

        temp_radar_coutour = radar_contour.copy()

        radar_contour = np.squeeze(radar_contour, -2)

        distances = (radar_contour[:, 0]) / 512. * jetyak_config['radar_distance']

        radar_indexes = radar_contour[:, 1]

        angles = radar_data['angle'][radar_indexes]

        utm_offsets = radar_data['utm'][radar_indexes]

        radar_contour = np.array([np.sin(angles) * distances, np.cos(angles) * distances]).T + utm_offsets

        if len(radar_contour) >= 3:

            radar_polygon = shapely.geometry.Polygon(radar_contour)

            if radar_polygon.is_valid:
                radar_polygons.append(radar_polygon)

            else:
                radar_polygon = radar_polygon.buffer(0)
                if not radar_polygon.is_empty:
                    if isinstance(radar_polygon, shapely.geometry.Polygon):
                        radar_polygons.append(radar_polygon)

                    else:
                        for single_radar_polygon in radar_polygon:
                            radar_polygons.append(single_radar_polygon)

    radar_polygons = shapely.ops.unary_union(radar_polygons)

    if isinstance(radar_polygons, shapely.geometry.Polygon):
        radar_polygons = shapely.geometry.MultiPolygon([radar_polygons])

    target_list = []

    for static_obstacle_interior in static_obstacle_polygon.interiors:

        static_interior_polygon = shapely.geometry.Polygon(static_obstacle_interior)

        if jetyak_point.within(static_interior_polygon):
            jetyak_interior_polygon = static_interior_polygon
            break

    else:
        jetyak_interior_polygon = shapely.geometry.Polygon()

    for radar_polygon in radar_polygons:

        if radar_polygon.area > 10:

            if radar_polygon.within(jetyak_interior_polygon):

                target_point = radar_polygon.centroid

                target_list.append({'z': np.array(target_point),
                                    'radius': target_point.hausdorff_distance(radar_polygon)})

                fill_polygon_alpha(radar_polygon, (225, 0, 0), 0.6)

            else:

                fill_polygon_alpha(radar_polygon, (220, 220, 220), 0.4)

    # global prev_target_list

    assoications = []
    assoications_pdf = []

    for target_index, target in enumerate(target_list):

        negative_index = -(target_index + 1)
        target_assoications = [negative_index]
        target_assoications_pdf = {negative_index: 0}

        for prev_target_index, prev_target in enumerate(prev_target_list):
            association_pdf = prev_target['pdf'].pdf(target['z'])

            if association_pdf > 0.00001:
                target_assoications.append(prev_target_index)
                target_assoications_pdf[prev_target_index] = association_pdf

        assoications.append(target_assoications)
        assoications_pdf.append(target_assoications_pdf)

    best_association_pdf_sum = -1
    best_association = []

    for assoication in itertools.product(*assoications):
        if len(set(assoication)) == len(assoication):

            assoication_pdf_sum = 0

            for target_index, prev_target_index in enumerate(assoication):
                assoication_pdf_sum += assoications_pdf[target_index][prev_target_index]

            if assoication_pdf_sum > best_association_pdf_sum:
                best_association_pdf_sum = assoication_pdf_sum
                best_association = assoication

    radar_filter = Kalman_filter.Radar_model(radar_dt)

    COLREGS_polygons = []

    for target_index, prev_target_index in enumerate(best_association):

        target = target_list[target_index]

        if prev_target_index < 0:

            target['x'] = np.array(
                [target['z'][0], target['z'][1], 0., 0.])

            target['P'] = np.diag(
                [1.**2, 1.**2, 1.5**2, 1.5**2])

            target['trajectory'] = []

        else:

            prev_target = prev_target_list[prev_target_index]

            prev_target['cache'] = -1

            target['x'], target['P'] = radar_filter.update(prev_target['x'], prev_target['P'], target['z'])

            target['trajectory'] = prev_target['trajectory'].copy()

            if len(target['trajectory']) > 2:

                is_ahead = abs(my_util.normalize_radian(np.arctan2(
                    target['x'][1] - jetyak_utm[1], target['x'][0] - jetyak_utm[0]) - jetyak_yaw)) < np.pi / 2

                if is_ahead:

                    target_yaw = np.arctan2(target['x'][3], target['x'][2])

                    yaw_diff = np.degrees(
                        my_util.normalize_radian(jetyak_yaw - target_yaw))

                    if yaw_diff < 0:
                        yaw_diff += 360

                    COLREGS_radius = cv2.norm(target['x'][2:]) * 5 + target['radius']

                    COLREGS_points = [target['x'][:2]]

                    is_COLREGS = True

                    if yaw_diff >= 165 and yaw_diff <= 195:
                        rotate_angles = np.linspace(-np.pi / 2,
                                                    0, 5) + target_yaw

                    elif yaw_diff > 195 and yaw_diff < 292.5:
                        rotate_angles = np.linspace(-np.pi / 4,
                                                    np.pi / 4, 5) + target_yaw

                    elif yaw_diff >= 292.5 or yaw_diff <= 67.5:
                        rotate_angles = np.linspace(0,
                                                    np.pi / 2, 5) + target_yaw

                    else:
                        is_COLREGS = False

                    if is_COLREGS:

                        for rotate_angle in rotate_angles:
                            COLREGS_points.append(my_util.get_rotate_points(
                                target['x'][:2], rotate_angle, COLREGS_radius))

                        COLREGS_polygon = shapely.geometry.Polygon(np.array(COLREGS_points))

                        COLREGS_polygons.append(COLREGS_polygon)

                        fill_polygon_alpha(COLREGS_polygon, (0, 0, 255), 0.6)

        target['trajectory'].append(target['x'][:2])

        cv2.polylines(plot_image, [my_util.utm_to_image(np.array(target['trajectory']), jetyak_utm, image_utm_ratio, half_image_dim)], False, (255, 0, 0), 2)

        target['x'], target['P'] = radar_filter.predict(target['x'], target['P'])

        target['pdf'] = scipy.stats.multivariate_normal(
            mean=target['x'][:2], cov=target['P'][:2, :2])

        target['cache'] = 1

    for prev_target in prev_target_list:

        if prev_target['cache'] != -1:

            prev_target['cache'] += 1

            if prev_target['cache'] <= cache_frames:
                target_list.append(prev_target)

    prev_target_list = target_list

    obstacle_polygons = shapely.ops.unary_union([static_obstacle_polygon, radar_polygons] + COLREGS_polygons)

    safe_distance = jetyak_config['length'] * 1.5

    obstacle_polygons = obstacle_polygons.buffer(safe_distance)

    obstacle_polygons = obstacle_polygons.difference(hole_polygons)

    if isinstance(obstacle_polygons, shapely.geometry.Polygon):
        obstacle_polygons = shapely.geometry.MultiPolygon([obstacle_polygons])

    # plot_image.fill(0)

    # for obstacle_polygon in obstacle_polygons:
    #     fill_polygon_alpha(obstacle_polygon, 255, 0)

    while len(input_waypoints) != 0:

        waypoint = input_waypoints[0]

        if cv2.norm(jetyak_utm, waypoint) < 5.:
            del input_waypoints[0]
        else:
            break

    waypoint_point = shapely.geometry.Point(waypoint)

    path = [jetyak_utm]

    if jetyak_point.within(obstacle_polygons):
        valid_jetyak_point = find_outside_point(jetyak_point, obstacle_polygons)
        valid_jetyak_utm = np.array(valid_jetyak_point)
        path.append(valid_jetyak_utm)

    else:
        valid_jetyak_point = jetyak_point
        valid_jetyak_utm = jetyak_utm

    jetyak_obstacle_polygon_interior = None
    waypoint_obstacle_polygon_interior = None

    for obstacle_polygon in obstacle_polygons:

        if waypoint_point.within(obstacle_polygon):
            raise Exception('waypoint inside')

        for obstacle_interior in obstacle_polygon.interiors:

            obstacle_polygon_interior = shapely.geometry.Polygon(obstacle_interior)

            jetyak_within_interior = valid_jetyak_point.within(obstacle_polygon_interior)
            waypoint_within_interior = waypoint_point.within(obstacle_polygon_interior)

            if jetyak_within_interior:
                jetyak_obstacle_polygon = shapely.geometry.Polygon(obstacle_polygon.exterior, holes=[obstacle_interior])
                jetyak_obstacle_polygon_interior = obstacle_polygon_interior
            if waypoint_within_interior:
                waypoint_obstacle_polygon_interior = obstacle_polygon_interior

        if jetyak_obstacle_polygon_interior is not None and waypoint_obstacle_polygon_interior is not None:
            break

    if jetyak_obstacle_polygon is not None:

        if jetyak_obstacle_polygon_interior == waypoint_obstacle_polygon_interior:
            valid_waypoint_point = waypoint_point
            valid_waypoint = waypoint

        else:
            valid_waypoint_point = shapely.ops.nearest_points(jetyak_obstacle_polygon_interior, waypoint_obstacle_polygon_interior)[0]
            valid_waypoint_point = find_outside_point(valid_waypoint_point, obstacle_polygons)
            valid_waypoint = np.array(valid_waypoint_point)

        current_obstacle_polygons = [jetyak_obstacle_polygon]

        for obstacle_polygon in obstacle_polygons:
            if obstacle_polygon.within(jetyak_obstacle_polygon.interiors[0]):
                current_obstacle_polygons.append(shapely.geometry.Polygon(obstacle_polygon.exterior))

        current_obstacle_polygons = shapely.geometry.MultiPolygon(current_obstacle_polygons)

        path_line = shapely.geometry.LineString([valid_jetyak_utm, valid_waypoint_point])

        intersect_lines = path_line.intersection(current_obstacle_polygons)

        intersect_points = []

        if not intersect_lines.is_empty:

            if isinstance(intersect_lines, shapely.geometry.LineString):
                intersect_points = intersect_lines

            else:
                for intersect_line in intersect_lines:
                    for intersect_point in intersect_line.coords:
                        intersect_points.append(intersect_point)

            intersect_points = np.insert(path_line, 1, intersect_points, axis=0)

            for intersect_index in range(3, len(intersect_points), 2):

                start_intersect_point = intersect_points[intersect_index - 2]
                end_intersect_point = intersect_points[intersect_index - 1]

                intersect_line = shapely.geometry.LineString([(intersect_points[intersect_index - 3] + start_intersect_point) / 2,
                                                              (end_intersect_point + intersect_points[intersect_index]) / 2])

                for current_obstacle_polygon in current_obstacle_polygons:
                    if intersect_line.intersects(current_obstacle_polygon):

                        split_polygons = list(shapely.ops.split(current_obstacle_polygon, intersect_line))

                        best_length = current_obstacle_polygon.length

                        for index, split_polygon in enumerate(split_polygons):
                            if split_polygon.length < best_length:
                                best_length < split_polygon.length
                                best_index = index

                        best_obstacle_contour = np.array(split_polygons[best_index].exterior)

                        start_contour_index = find_nearest_index(best_obstacle_contour, start_intersect_point)
                        end_contour_index = find_nearest_index(best_obstacle_contour, end_intersect_point)

                        len_best_obstacle_contour = len(best_obstacle_contour)
                        abs_start_end_index = abs(start_contour_index - end_contour_index)

                        if (start_contour_index < end_contour_index and abs_start_end_index < len_best_obstacle_contour - abs_start_end_index) or \
                                (start_contour_index > end_contour_index and abs_start_end_index > len_best_obstacle_contour - abs_start_end_index):
                            best_obstacle_contour = np.flip(best_obstacle_contour, axis=0)

                            start_contour_index = len_best_obstacle_contour - 1 - start_contour_index
                            end_contour_index = len_best_obstacle_contour - 1 - end_contour_index

                        best_obstacle_contour = np.roll(best_obstacle_contour, -start_contour_index, axis=0)

                        best_obstacle_contour = best_obstacle_contour[:end_contour_index - start_contour_index + 1]

                        path.extend(best_obstacle_contour)
                        break

        path.append(valid_waypoint)

        path = cv2.approxPolyDP(my_util.get_rint(path), 2., False)
        path = np.squeeze(path, -2)

        start_index = 0

        while start_index < len(path) - 2:
            start_point = path[start_index]
            end_index = len(path) - 1
            while end_index > start_index + 1:
                end_point = path[end_index]
                path_line = shapely.geometry.LineString([start_point, end_point])
                if not path_line.intersects(current_obstacle_polygons):
                    path = np.delete(path, range(
                        start_index + 1, end_index), 0)
                    break
                end_index -= 1
            start_index += 1

    cv2.polylines(plot_image, [my_util.utm_to_image(path, jetyak_utm, image_utm_ratio, half_image_dim)],
                  False, (247, 220, 111), 4)

    path = path[1:]

    # update_waypoints(path)

    for point in path:
        cv2.circle(plot_image, tuple(my_util.utm_to_image(point, jetyak_utm, image_utm_ratio, half_image_dim)),
                   8, (241, 196, 15), -1)
    # else:
    #     clear_waypoints()

    cv2.imwrite('test_images/' + str(time_stamp.secs) + '.png',
                cv2.cvtColor(np.flipud(plot_image), cv2.COLOR_RGB2BGR))

    end = time.time()
    print(time_stamp.secs)
    print('Time:', end - start)
    print('-----------------------------------------')


# process(radar_data, jetyak_config, rospy.Time.from_sec(0))
