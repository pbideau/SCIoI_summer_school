# This file contains functions unchanged from tutorial-1

import numpy as np

# Logic from https://github.com/scipy/scipy/blob/a4ed9450b6d5c1f36495b29a0471f03b471ca8ae/scipy/spatial/_plotutils.py#L235-L254
def extract_line_segments(vor):
    center = vor.points.mean(axis=0)
    ptp_bound = np.ptp(vor.points, axis=0)

    segments = []
    for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
        simplex = np.asarray(simplex)
        if np.all(simplex >= 0):
            segments.append(vor.vertices[simplex])
        else:
            i = simplex[simplex >= 0][0]  # finite end Voronoi vertex

            t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # tangent
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])  # normal

            midpoint = vor.points[pointidx].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            if (vor.furthest_site):
                direction = -direction
            far_point = vor.vertices[i] + direction * ptp_bound.max()

            segments.append([vor.vertices[i], far_point])
    return segments

def extract_line_segment(vor, idx):
    center = vor.points.mean(axis=0)
    ptp_bound = np.ptp(vor.points, axis=0)

    pointidx, simplex = vor.ridge_points[idx], vor.ridge_vertices[idx]
    simplex = np.asarray(simplex)
    if np.all(simplex >= 0):
        return vor.vertices[simplex]
    else:
        i = simplex[simplex >= 0][0]  # finite end Voronoi vertex

        t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # tangent
        t /= np.linalg.norm(t)
        n = np.array([-t[1], t[0]])  # normal

        midpoint = vor.points[pointidx].mean(axis=0)
        direction = np.sign(np.dot(midpoint - center, n)) * n
        if (vor.furthest_site):
            direction = -direction
        far_point = vor.vertices[i] + direction * ptp_bound.max()

        return [vor.vertices[i], far_point]

def line_segment_to_hyperspace(point, ls):
    direction = ls[0] - ls[1]
    direction /= np.linalg.norm(direction)
    n = np.array([-direction[1], direction[0]])
    # compute the distance
    d = np.dot(ls[0], n)
    # check the direction with respect to robot point
    if np.dot(n, point) < d:
        n = -n
        d = -d
    return n, d

def extract_hyperspaces_per_point(vor):
    result = []
    # segments = extract_line_segments(vor)
    # # print("seg", segments)
    # for point, region_idx in zip(vor.points, vor.point_region):
    #     result_per_point = []
    #     for region in vor.regions[region_idx]:
    #         ls = extract_line_segment(vor, region)
    #         hs = line_segment_to_hyperspace(point, ls)
    #         print(point, hs)
    #         result_per_point.append(hs)
    #     result.append(result_per_point)
    segments = extract_line_segments(vor)
    for pidx, point in enumerate(vor.points):
        result_per_point = []
        for (pidx1, pidx2), ls in zip(vor.ridge_points, segments):
            if pidx == pidx1 or pidx == pidx2:
                hs = line_segment_to_hyperspace(point, ls)
                result_per_point.append(hs)
        result.append(result_per_point)
    return result
