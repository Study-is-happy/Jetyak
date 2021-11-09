import numpy as np
import shapely
import geopandas
import matplotlib.pyplot as plt

import my_util

shape_files_dir = 'shape_files/'


def get_polygon(file_path):

    utm_list = []

    with open(file_path) as land_file:

        land_file.readline()

        for gps in land_file:
            gps = np.float64(gps.split(','))
            utm = my_util.latlng_to_utm(gps)
            utm_list.append(utm)

    polygon = shapely.geometry.Polygon(utm_list)
    # print(len(polygon.exterior.coords))
    # print(polygon.area)

    # polygon = polygon.simplify(0.1, preserve_topology=False)

    # print(len(polygon.exterior.coords))
    # print(polygon.area)

    return polygon


gdf = geopandas.GeoSeries([get_polygon(shape_files_dir + 'land_1.txt'),
                           get_polygon(shape_files_dir + 'land_2.txt')])
gdf.to_file(shape_files_dir + 'lands.shp')
gdf.plot()
plt.show()
