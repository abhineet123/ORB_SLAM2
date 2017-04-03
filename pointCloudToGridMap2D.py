import numpy as np
import cv2
import sys
from transforms3d import quaternions


def get_line_bresenham(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


seq_name = 'kitti_00'
# seq_name = 'tum'
# inverse of cell size
scale_factor = 3
resize_factor = 5
filter_ground_points = 0
load_counters = 0

point_cloud_fname = '{:s}_map_pts_and_keyframes.txt'.format(seq_name)
keyframe_trajectory_fname = '{:s}_key_frame_trajectory.txt'.format(seq_name)
visit_counter_fname = '{:s}_filtered_{:d}_scale_{:d}_visit_counter.txt'.format(
    seq_name, filter_ground_points, scale_factor)
occupied_counter_fname = '{:s}_filtered_{:d}_scale_{:d}_occupied_counter.txt'.format(
    seq_name, filter_ground_points, scale_factor)

print 'seq_name: ', seq_name
print 'scale_factor: ', scale_factor
print 'resize_factor: ', resize_factor
print 'filter_ground_points: ', filter_ground_points

counters_loaded = False
if load_counters:
    try:
        print 'Loading counters...'
        visit_counter = np.loadtxt(visit_counter_fname)
        occupied_counter = np.loadtxt(occupied_counter_fname)
        grid_res = visit_counter.shape
        print 'grid_res: ', grid_res
        counters_loaded = True
    except:
        print 'One or more counter files: {:s}, {:s} could not be found'.format(
            occupied_counter_fname, visit_counter_fname)
        counters_loaded = False

if not counters_loaded:
    # keyframe_trajectory_data = np.loadtxt(keyframe_trajectory_fname, dtype=np.float32)
    # keyframe_timestamps = keyframe_trajectory_data[:, 0]
    # keyframe_locations = keyframe_trajectory_data[:, 1:4]
    # keyframe_quaternions = keyframe_trajectory_data[:, 5:9]

    # read keyframes
    keyframe_trajectory_data = open(keyframe_trajectory_fname, 'r').readlines()
    keyframe_timestamps = []
    keyframe_locations = []
    keyframe_quaternions = []
    for line in keyframe_trajectory_data:
        line_tokens = line.strip().split()
        timestamp = float(line_tokens[0])
        keyframe_x = float(line_tokens[1]) * scale_factor
        keyframe_y = float(line_tokens[2]) * scale_factor
        keyframe_z = float(line_tokens[3]) * scale_factor
        keyframe_timestamps.append(timestamp)
        keyframe_locations.append([keyframe_x, keyframe_y, keyframe_z])
        keyframe_quaternions.append([float(line_tokens[4]), float(line_tokens[5]),
                                     float(line_tokens[6]), float(line_tokens[7])])

    keyframe_locations_dict = dict(zip(keyframe_timestamps, keyframe_locations))
    keyframe_quaternions_dict = dict(zip(keyframe_timestamps, keyframe_quaternions))
    keyframe_locations = np.array(keyframe_locations)
    keyframe_timestamps = np.array(keyframe_timestamps)
    n_keyframes = keyframe_locations.shape[0]

    # read point cloud
    point_cloud_data = open(point_cloud_fname, 'r').readlines()
    point_locations = []
    point_timestamps = []
    n_lines = len(point_cloud_data)
    n_points = 0
    is_ground_point = []
    for line in point_cloud_data:
        line_tokens = line.strip().split()
        point_x = float(line_tokens[0]) * scale_factor
        point_y = float(line_tokens[1]) * scale_factor
        point_z = float(line_tokens[2]) * scale_factor

        timestamps = []

        for i in xrange(3, len(line_tokens)):
            timestamps.append(float(line_tokens[i]))
        if len(timestamps) == 0:
            raise StandardError('Point {:d} has no keyframes'.format(n_points))

        is_ground_point.append(False)

        if filter_ground_points:
            key_frame_id = 0
            keyframe_quaternion = None
            keyframe_location = None
            while key_frame_id < len(timestamps):
                try:
                    keyframe_quaternion = np.array(keyframe_quaternions_dict[timestamps[key_frame_id]])
                    keyframe_location = keyframe_locations_dict[timestamps[key_frame_id]]
                    break
                except KeyError:
                    key_frame_id += 1
            if keyframe_quaternion is None:
                raise StandardError('No valid keyframes found for point {:d}'.format(n_points))
            # normalize quaternion
            keyframe_quaternion /= np.linalg.norm(keyframe_quaternion)
            keyframe_rotation = quaternions.quat2mat(keyframe_quaternion)
            keyframe_translation = np.matrix(keyframe_location).transpose()
            transform_mat = np.zeros([4, 4], dtype=np.float64)
            transform_mat[0:3, 0:3] = keyframe_rotation.transpose()
            transform_mat[3, 0:3] = (np.matrix(-keyframe_rotation.transpose()) * keyframe_translation).ravel()
            transform_mat[3, 3] = 1
            point_location = np.matrix([point_x, point_y, point_z, 1]).transpose()
            transformed_point_location = np.matrix(transform_mat) * point_location
            # homogeneous to non homogeneous coordinates
            transformed_point_height = transformed_point_location[1] / transformed_point_location[3]
            if transformed_point_height < 0:
                is_ground_point[-1] = True

        point_locations.append([point_x, point_y, point_z])
        point_timestamps.append(timestamps)
        n_points += 1

    point_locations = np.array(point_locations)
    print 'n_keyframes: ', n_keyframes
    print 'n_points: ', n_points
    if filter_ground_points:
        n_ground_points = np.count_nonzero(np.array(is_ground_point))
        print 'n_ground_points: ', n_ground_points

    kf_min_x = np.floor(np.min(keyframe_locations[:, 0]))
    kf_min_z = np.floor(np.min(keyframe_locations[:, 2]))
    kf_max_x = np.ceil(np.max(keyframe_locations[:, 0]))
    kf_max_z = np.ceil(np.max(keyframe_locations[:, 2]))

    pc_min_x = np.floor(np.min(point_locations[:, 0]))
    pc_min_z = np.floor(np.min(point_locations[:, 2]))
    pc_max_x = np.ceil(np.max(point_locations[:, 0]))
    pc_max_z = np.ceil(np.max(point_locations[:, 2]))

    grid_min_x = min(kf_min_x, pc_min_x)
    grid_min_z = min(kf_min_z, pc_min_z)
    grid_max_x = max(kf_max_x, pc_max_x)
    grid_max_z = max(kf_max_z, pc_max_z)

    print 'grid_max_x: ', grid_max_x
    print 'grid_min_x: ', grid_min_x
    print 'grid_max_z: ', grid_max_z
    print 'grid_min_z: ', grid_min_z

    grid_res = [int(grid_max_x - grid_min_x), int(grid_max_z - grid_min_z)]
    print 'grid_res: ', grid_res

    visit_counter = np.zeros(grid_res, dtype=np.int32)
    occupied_counter = np.zeros(grid_res, dtype=np.int32)

    print 'grid extends from ({:f}, {:f}) to ({:f}, {:f})'.format(
        grid_min_x, grid_min_z, grid_max_x, grid_max_z)

    grid_cell_size_x = (grid_max_x - grid_min_x) / float(grid_res[0])
    grid_cell_size_z = (grid_max_z - grid_min_z) / float(grid_res[1])

    # print 'using cell size: {:f} x {:f}'.format(grid_cell_size_x, grid_cell_size_z)

    norm_factor_x = float(grid_res[0] - 1) / float(grid_max_x - grid_min_x)
    norm_factor_z = float(grid_res[1] - 1) / float(grid_max_z - grid_min_z)
    print 'norm_factor_x: ', norm_factor_x
    print 'norm_factor_z: ', norm_factor_z

    # print 'len(is_ground_point):', len(is_ground_point)

    for point_id in xrange(n_points):
        point_location = point_locations[point_id]
        for timestamp in point_timestamps[point_id]:
            try:
                keyframe_location = keyframe_locations_dict[timestamp]
            except KeyError:
                print 'Timestamp: {:f} not found'.format(timestamp)
                continue
            keyframe_x = int(keyframe_location[0])
            keyframe_z = int(keyframe_location[2])
            point_x = int(point_location[0])
            point_z = int(point_location[2])
            ray_points = get_line_bresenham([keyframe_x, keyframe_z], [point_x, point_z])
            n_ray_pts = len(ray_points)

            for ray_point_id in xrange(n_ray_pts - 1):
                ray_point_x_norm = int(np.floor((ray_points[ray_point_id][0] - grid_min_x) * norm_factor_x))
                ray_point_z_norm = int(np.floor((ray_points[ray_point_id][1] - grid_min_z) * norm_factor_z))
                # start_x = ray_point_x_norm - resize_factor / 2
                # start_z = ray_point_z_norm - resize_factor / 2
                # end_x = ray_point_x_norm + resize_factor / 2
                # end_z = ray_point_z_norm + resize_factor / 2
                try:
                    # visit_counter[start_x:end_x, start_z:end_z] += 1
                    visit_counter[ray_point_x_norm, ray_point_z_norm] += 1
                except IndexError:
                    print 'Out of bound point: ({:d}, {:d}) -> ({:f}, {:f})'.format(
                        ray_points[ray_point_id][0], ray_points[ray_point_id][1],
                        ray_point_x_norm, ray_point_z_norm)
                    sys.exit(0)
            ray_point_x_norm = int(np.floor((ray_points[-1][0] - grid_min_x) * norm_factor_x))
            ray_point_z_norm = int(np.floor((ray_points[-1][1] - grid_min_z) * norm_factor_z))
            # start_x = ray_point_x_norm - resize_factor / 2
            # start_z = ray_point_z_norm - resize_factor / 2
            # end_x = ray_point_x_norm + resize_factor / 2
            # end_z = ray_point_z_norm + resize_factor / 2
            try:
                if is_ground_point[point_id]:
                    visit_counter[ray_point_x_norm, ray_point_z_norm] += 1
                else:
                    # occupied_counter[start_x:end_x, start_z:end_z] += 1
                    occupied_counter[ray_point_x_norm, ray_point_z_norm] += 1
            except IndexError:
                print 'Out of bound point: ({:d}, {:d}) -> ({:f}, {:f})'.format(
                    ray_points[-1][0], ray_points[-1][1], ray_point_x_norm, ray_point_z_norm)
                sys.exit(0)
        if (point_id + 1) % 1000 == 0:
            print 'Done {:d} points of {:d}'.format(point_id + 1, n_points)

    print 'Saving counters to {:s} and {:s}'.format(occupied_counter_fname, visit_counter_fname)
    np.savetxt(occupied_counter_fname, occupied_counter, fmt='%d')
    np.savetxt(visit_counter_fname, visit_counter, fmt='%d')

# occupied_counter_zeros = occupied_counter == 0
# occupied_counter[occupied_counter_zeros] = 2 * visit_counter[occupied_counter_zeros]
# grid_map = visit_counter.astype(np.float32) / occupied_counter.astype(np.float32)

free_thresh = 0.55
occupied_thresh = 0.50

grid_map = np.zeros(grid_res, dtype=np.float32)
grid_map_thresh = np.zeros(grid_res, dtype=np.uint8)
for x in xrange(grid_res[0]):
    for z in xrange(grid_res[1]):
        if visit_counter[x, z] == 0 or occupied_counter[x, z] == 0:
            grid_map[x, z] = 0.5
        else:
            grid_map[x, z] = 1 - occupied_counter[x, z] / visit_counter[x, z]
        if grid_map[x, z] >= free_thresh:
            grid_map_thresh[x, z] = 255
        elif occupied_thresh <= grid_map[x, z] < free_thresh:
            grid_map_thresh[x, z] = 128
        else:
            grid_map_thresh[x, z] = 0

if resize_factor != 1:
    grid_res_resized = (grid_res[0] * resize_factor, grid_res[1] * resize_factor)
    print 'grid_res_resized: ', grid_res_resized
    grid_map_resized = cv2.resize(grid_map_thresh, grid_res_resized)
else:
    grid_map_resized = grid_map_thresh

out_fname = 'grid_map_{:s}_filtered_{:d}_scale_{:d}_resize_{:d}_{:.2f}_{:.2f}'.format(
    seq_name, filter_ground_points, scale_factor, resize_factor, occupied_thresh, free_thresh)
cv2.imwrite('{:s}.pgm'.format(out_fname), grid_map_resized)
cv2.imshow(out_fname, grid_map_resized)
cv2.waitKey(0)






















