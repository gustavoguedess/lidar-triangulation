from aux.simtwo import SimTwo
from math import atan2
import math

DEBUG = False
BREAKPOINT = False

INITIAL_AGV_X = 0
INITIAL_AGV_Y = 0
INITIAL_AGV_ANGLE = 0

LEFT_X = - 1.7/2 - 0.05
RIGHT_X = 1.7/2 + 0.05
UPPER_Y = 1.2/2 + 0.05
BOTTOM_Y = - 1.2/2 - 0.05

LIMIT_DIFF_OBJECT = 0.2
MIN_SIZE = 5
MAX_SIZE = 40
MIN_DIST = 0.05
MAX_DIST = 2
LIMIT_GAP = 3

SAMPLES = 360

st = SimTwo()

first_beacon_angle = 20

## AUX FUNCTIONS ##
def remove_outliers(data):
    data = data.copy()

    for i in range(len(data)):
        if   data[i] < MIN_DIST: data[i] = 0
        elif data[i] > MAX_DIST: data[i] = MAX_DIST
    
    return data

def angle_sum(a, b):
    return (a+b+360) % 360

def angle_sub(a, b):
    return (a-b+360) % 360

def angle_diff(a, b):
    return min(angle_sub(a, b), angle_sub(b, a))

### GROUPING OBJECTS ###
def dumb_group(data):
    groups = []
    i = 0
    while i < len(data):
        if data[i] != 0:
            start_object = i
            end_object = i+1
            while end_object < len(data) and data[end_object] != 0 and abs(data[end_object]-data[end_object-1]) < LIMIT_DIFF_OBJECT:
                end_object += 1
            groups.append((start_object, end_object))
            i = end_object
        else:
            i += 1
    return groups

def union_similar_objs(data, groups):
    groups = groups.copy()

    is_same_size = lambda i, j: abs(data[groups[i][1]-1]-data[groups[j][0]]) < LIMIT_DIFF_OBJECT
    is_near = lambda i, j: j-i < LIMIT_GAP if i < j else len(data)-i+j < LIMIT_GAP
    i = 0
    while i < len(groups):
        if is_same_size(i, (i+1)%len(groups)) and is_near(groups[i][1], groups[(i+1)%len(groups)][0]):
            groups[i] = (groups[i][0], groups[(i+1)%len(groups)][1])
            groups.pop((i+1)%len(groups))
        else:
            i += 1
    return groups

def get_distance_group(data, start, end):
    return data[(start+end)//2]
# Return a list of objects range detected by the lidar sensor
def group_data(data):
    ranges = dumb_group(data)
    ranges = union_similar_objs(data, ranges)
    #TODO I think I can merge these two functions

    groups = []
    for _start, _end in ranges:
        obj = {
            'start': _start,
            'end': _end,
            'middle': ((_start+_end)//2)%360,
            'size': abs(_end-_start),
            'distance': get_distance_group(data, _start, _end) + 0.055, #TODO magic number is equivalent to the beacon radius
            'data': data[_start:_end] if _start < _end else data[_start:] + data[:_end]
        }
        groups.append(obj)
    return groups


### BEACONS ###

def get_gamma_beacon(trilateration, bx, by):
    dy = by - trilateration['y']
    dx = bx - trilateration['x']
    gamma = atan2(dy, dx)
    gamma = gamma * 180 / math.pi

    gamma = (gamma + 360) % 360
    return gamma

def euclidean_distance(*args):
    if len(args) == 2:
        a_x, a_y, b_x, b_y = args[0]['x'], args[0]['y'], args[1]['x'], args[1]['y']
    elif len(args) == 4:
        a_x, a_y, b_x, b_y = args
    else: return None

    return math.sqrt((a_x-b_x)**2 + (a_y-b_y)**2)

def get_similar_beacon(objs, beacon):
    possible_beacons = []

    for obj in objs:
        # obj['distance'] = calc_distance_beacon(obj)
        # error_distance = error_distance_beacon(beacon)
        # error_phi = error_phi_beacon(beacon)

        # if abs(obj['distance'] - beacon['distance']) < error_distance \
        #     and angle_diff(beacon['phi'], np.radians(obj['middle'])) < error_phi:
        #     possible_beacons.append(obj)

        diff_phi = angle_diff(beacon['phi'], np.radians(obj['middle']))
        diff_distance = abs(beacon['distance'] - obj['distance'])
        #TODO: Improve this get similar beacon function
        if diff_phi < 0.5 and diff_distance < 0.5:
            possible_beacons.append(obj)    

    if len(possible_beacons) == 1:
        return possible_beacons[0]
    
    return None

def calc_phi(trilateration, bx, by):
    dy = by - trilateration['y']
    dx = bx - trilateration['x']
    gamma = atan2(dy, dx)
    theta = trilateration['theta'] if trilateration['theta'] >= 0 else 360 + trilateration['theta']
    theta = math.radians(theta)
    phi = diff_angle(gamma, theta)
    return phi

def calc_beacon(trilateration, bx, by):
    return {
        'x': bx,
        'y': by,
        'phi': calc_phi(trilateration, bx, by),
        'distance': euclidean_distance(trilateration['x'], trilateration['y'], bx, by),
    }

def calc_beacons(trilateration):
    beacons = {
        'beacon1': calc_beacon(trilateration, LEFT_X, UPPER_Y),
        'beacon2': calc_beacon(trilateration, LEFT_X, BOTTOM_Y),
        'beacon3': calc_beacon(trilateration, RIGHT_X, BOTTOM_Y),
        'beacon4': calc_beacon(trilateration, RIGHT_X, UPPER_Y),
    }
    return beacons

def obj_to_beacon(obj, last_beacon, last_trilateration=None):
    #TODO improve this function
    if obj: 
        distance = obj['distance']+0.055
        phi = obj['middle']
    else:
        distance = euclidean_distance(last_beacon['x'], last_beacon['y'], last_trilateration['x'], last_trilateration['y'])
        phi = last_beacon['phi']
    
    return {
        'x': last_beacon['x'],
        'y': last_beacon['y'],
        'phi': phi,
        'distance': distance,
    }

def get_beacons(objs, last_trilateration):
    last_beacons = calc_beacons(last_trilateration)
    
    obj1 = get_similar_beacon(objs, last_beacons['beacon1'])
    obj2 = get_similar_beacon(objs, last_beacons['beacon2'])
    obj3 = get_similar_beacon(objs, last_beacons['beacon3'])
    obj4 = get_similar_beacon(objs, last_beacons['beacon4'])

    beacons = {
        'beacon1': obj_to_beacon(obj1, last_beacons['beacon1'], last_trilateration),
        'beacon2': obj_to_beacon(obj2, last_beacons['beacon2'], last_trilateration),
        'beacon3': obj_to_beacon(obj3, last_beacons['beacon3'], last_trilateration),
        'beacon4': obj_to_beacon(obj4, last_beacons['beacon4'], last_trilateration),
    }

    return beacons

### TRILATERATION ###

# b1.y = b2.y && b1.x = b3.x
def calc_position(b1, b2, b3):
    x = (b1['distance']**2 - b2['distance']**2 + b2['x']**2 - b1['x']**2 + b2['y']**2 - b1['y']**2) / (2 * (b2['x'] - b1['x']))
    y = (b1['distance']**2 - b3['distance']**2 + b3['x']**2 - b1['x']**2 + b3['y']**2 - b1['y']**2) / (2 * (b3['y'] - b1['y'])) - (b3['x'] - b1['x']) / (b3['y'] - b1['y']) * x
    return x, y

# Return the y position of the beacon. The both beacons must have the same x position
def calc_y(b1,b2):
    if 'distance' not in b1 or 'distance' not in b2: return None
    return (b1['distance']**2 - b2['distance']**2 + b2['y']**2 - b1['y']**2) / (2*b2['y'] - 2*b1['y'])

# Return the x position of the beacon. The both beacons must have the same y position
def calc_x(b1,b2):
    if 'distance' not in b1 or 'distance' not in b2: return None
    return (b1['distance']**2 - b2['distance']**2 + b2['x']**2 - b1['x']**2) / (2*b2['x'] - 2*b1['x'])



def get_position(beacons):
    # x1, y1 = calc_position(beacons['beacon3'], beacons['beacon1'], beacons['beacon2'])
    # return x1, y1

    
    x1 = calc_x(beacons['beacon2'], beacons['beacon3']) if 'beacon2' in beacons and 'beacon3' in beacons else None
    x2 = calc_x(beacons['beacon1'], beacons['beacon4']) if 'beacon1' in beacons and 'beacon4' in beacons else None
    y1 = calc_y(beacons['beacon2'], beacons['beacon1']) if 'beacon2' in beacons and 'beacon1' in beacons else None
    y2 = calc_y(beacons['beacon3'], beacons['beacon4']) if 'beacon3' in beacons and 'beacon4' in beacons else None

    x = [x1, x2] 
    y = [y1, y2]

    # remove if there is none value
    x = [i for i in x if i]
    y = [i for i in y if i]
    
    if len(x) == 0 or len(y) == 0: return None, None

    x = sum(x) / len(x)
    y = sum(y) / len(y)

    return x, y

def get_angle(x, y, xi, yi):
    dy = yi - y
    dx = xi - x
    theta = atan2(dy, dx)
    theta = theta * 180 / math.pi
    theta = (theta + 360) % 360
    
    return theta

import numpy as np

def diff_angle(angle1, angle2):
    return (angle1 - angle2 + 2 * np.pi ) % (2 * np.pi)

def angle_mean(angles):
    x = np.mean(np.cos(angles))
    y = np.mean(np.sin(angles))
    theta = np.arctan2(y, x)
    theta_normalized = (theta + 2 * np.pi) % (2 * np.pi)
    return theta_normalized

#TODO Verify later, this function is not working properly
def asin(x):
    if x < -1: x = -1
    if x > 1: x = 1
    result = np.arcsin( x )
    result = (result + 2 * np.pi) % (2 * np.pi)
    return result

def calc_theta(beacons):
    # distances
    dist_12 = euclidean_distance(beacons['beacon1'], beacons['beacon2'])
    dist_23 = euclidean_distance(beacons['beacon2'], beacons['beacon3'])
    dist_34 = euclidean_distance(beacons['beacon3'], beacons['beacon4'])
    dist_41 = euclidean_distance(beacons['beacon4'], beacons['beacon1'])

    dist_1 = beacons['beacon1']['distance'] if 'distance' in beacons['beacon1'] else 0
    dist_2 = beacons['beacon2']['distance'] if 'distance' in beacons['beacon2'] else 0
    dist_3 = beacons['beacon3']['distance'] if 'distance' in beacons['beacon3'] else 0
    dist_4 = beacons['beacon4']['distance'] if 'distance' in beacons['beacon4'] else 0

    # phi
    phi_1 = np.radians(beacons['beacon1']['phi']) if 'phi' in beacons['beacon1'] else 0
    phi_2 = np.radians(beacons['beacon2']['phi']) if 'phi' in beacons['beacon2'] else 0
    phi_3 = np.radians(beacons['beacon3']['phi']) if 'phi' in beacons['beacon3'] else 0
    phi_4 = np.radians(beacons['beacon4']['phi']) if 'phi' in beacons['beacon4'] else 0

    # diff phi between beacons
    delta_12 = diff_angle(phi_2, phi_1)
    delta_23 = diff_angle(phi_3, phi_2)
    delta_34 = diff_angle(phi_4, phi_3)
    delta_41 = diff_angle(phi_1, phi_4)
    
    # gamma
    b1_gamma   =  (np.pi/2   + asin( np.sin(delta_12) * dist_2 / dist_12 ) ) % (2*np.pi)
    b2_gamma   =  (np.pi     + asin( np.sin(delta_23) * dist_3 / dist_23 ) ) % (2*np.pi)
    b3_gamma   =  (3*np.pi/2 + asin( np.sin(delta_34) * dist_4 / dist_34 ) ) % (2*np.pi)
    b4_gamma   =  (            asin( np.sin(delta_41) * dist_1 / dist_41 ) ) % (2*np.pi)
    b1_gamma_l =  (np.pi     - asin( np.sin(delta_41) * dist_4 / dist_41 ) + 2*np.pi) % (2*np.pi)
    b2_gamma_l =  (3*np.pi/2 - asin( np.sin(delta_12) * dist_1 / dist_12 ) + 2*np.pi) % (2*np.pi)
    b3_gamma_l =  (2*np.pi   - asin( np.sin(delta_23) * dist_2 / dist_23 ) + 2*np.pi) % (2*np.pi)
    b4_gamma_l =  (np.pi/2   - asin( np.sin(delta_34) * dist_3 / dist_34 ) + 2*np.pi) % (2*np.pi)

    # theta
    b1_theta    = diff_angle(b1_gamma   , phi_1)
    b2_theta    = diff_angle(b2_gamma   , phi_2)
    b3_theta    = diff_angle(b3_gamma   , phi_3)
    b4_theta    = diff_angle(b4_gamma   , phi_4)
    b1_theta_l  = diff_angle(b1_gamma_l , phi_1)
    b2_theta_l  = diff_angle(b2_gamma_l , phi_2)
    b3_theta_l  = diff_angle(b3_gamma_l , phi_3)
    b4_theta_l  = diff_angle(b4_gamma_l , phi_4)
    
    theta_list = [b1_theta, b2_theta, b3_theta, b4_theta, b1_theta_l, b2_theta_l, b3_theta_l, b4_theta_l]

    # degrees for debug
    b1_theta_deg = np.degrees(b1_theta)
    b2_theta_deg = np.degrees(b2_theta)
    b3_theta_deg = np.degrees(b3_theta)
    b4_theta_deg = np.degrees(b4_theta)
    b1_theta_l_deg = np.degrees(b1_theta_l)
    b2_theta_l_deg = np.degrees(b2_theta_l)
    b3_theta_l_deg = np.degrees(b3_theta_l)
    b4_theta_l_deg = np.degrees(b4_theta_l)

    theta_rad = angle_mean(theta_list)

    theta_degree = np.degrees(theta_rad)

    theta = theta_degree if theta_degree < 180 else theta_degree - 360

    if theta is None: 
        return None
    return theta

def get_theta(x, y, beacons):
    theta_list = []
    
    for beacon in beacons.values():
        if 'phi' not in beacon: continue
        phi = beacon['phi']
        gamma = beacon['last_gamma']
        
        theta = angle_sub(gamma, phi)
        if len(theta_list) > 0 and theta-theta_list[-1] > 30: theta -= 360
        if len(theta_list) > 0 and theta-theta_list[-1] < -30: theta += 360
        theta_list.append(theta)
    
    theta = sum(theta_list) / len(theta_list)
    theta = (theta + 360) % 360
    

    return theta

def get_trilateration(data_raw, last_trilateration=None):
    if last_trilateration['theta'] < 0: last_trilateration['theta'] = 360 + last_trilateration['theta']

    # Remove outliers
    data = remove_outliers(data_raw)
    if not any(data): return last_trilateration
    
    # Groupping objects
    objs_groups = group_data(data)

    # Get beacons
    beacons = get_beacons(objs_groups, last_trilateration)
    
    # if there is fewer than 3 beacons, return the last trilateration
    if len(beacons) < 3: return last_trilateration

    # Get position and angle from beacons
    x,y = get_position(beacons)
    theta = calc_theta(beacons)

    trilateration = {
        'x': x,
        'y': y,
        'theta': theta,
    }
    
    if trilateration['theta'] > 180: trilateration['theta'] = trilateration['theta']-360

    return trilateration

#################### INPUT ####################

def get_initial_trilateration():
    data = get_data()
    
    x, y, theta = data['ground_truth'].values()

    # trilateration = {
    #     'x': INITIAL_AGV_X,
    #     'y': INITIAL_AGV_Y,
    #     'theta': INITIAL_AGV_ANGLE,
    # }
    trilateration = {
        'x': x,
        'y': y,
        'theta': theta,
    }

    return trilateration

generator_data = st.gen_data()
def get_data():
    # GET DATA FROM SIMULATION
    return st.get_data()

    # GET DATA FROM FILE
    # try:
    #     return generator_data.__next__()
    # except StopIteration:
    #     return None

#################### DEBUG ####################

def print_lidar_data(lidar_data):
    print('Lidar data:', end=' ')
    for data in lidar_data:
        print(f'{data:0.2f}', end=' ')
    print('\n')

def print_objs(objs):
    print('Total Objs detecteds: ', len(objs))
    for obj in objs:
        print(f'Obj: [{obj["start"]}, {obj["end"]}] Size: {obj["size"]} Distance: {obj["distance"]}', end=' ' )
        for data in obj['data']:
            print(f'{data:0.2f}', end=' ')
        print()
    print()

def print_beacons(beacons):
    print('Total Beacons detecteds: ', len(beacons))
    for name, value in beacons.items():
        print(f'{name}: ', end='')
        print(f'X: {value["x"]:.2f}', end=' ')
        print(f'Y: {value["y"]:.2f}', end=' ')
        if 'distance' in value:
            print(f'Phi: {value["phi"]:.2f}', end=' ')
            print(f'Phi Points: {value["phi_points"]:.2f}', end=' ')
            print(f'Distance: {value["distance"]:.2f}', end=' ')
            print(f'Distance Points: {value["distance_points"]:.2f}', end=' ')
            print(f'Similarity Points: {value["similarity_points"]:.2f}')

    print()

def print_trilateration(trilateration, ground_truth=None):
    print(f'Trilateration: ({trilateration["x"]:.2f}, {trilateration["y"]:.2f}, {trilateration["theta"]:.2f})')
    if ground_truth:
        print(f'Ground Truth:  ({ground_truth["x"]:.2f}, {ground_truth["y"]:.2f}, {ground_truth["theta"]:.2f}', end=' ')

def print_ground_truth(ground_truth):
    print('Ground Truth:  ', end='')
    print(f'X: {ground_truth["x"]:.2f}', end=' ')
    print(f'Y: {ground_truth["y"]:.2f}', end=' ')
    print(f'Theta: {ground_truth["theta"]:.2f}')

def print_data(data):
    print_lidar_data(data['raw_lidar'])
    print_trilateration(data['trilateration'])
    

def save_trilateration(trilateration, ground_truth=None):
    with open('data/trilateration.txt', 'w') as f:
        f.write(f'{trilateration["x"]:.2f} {trilateration["y"]:.2f} {trilateration["theta"]:.2f}\n')
        if ground_truth:
            f.write(f'{ground_truth["x"]:.2f} {ground_truth["y"]:.2f} {ground_truth["theta"]:.2f}')
    
def save_data(data):
    with open('data/data.txt', 'w') as f:
        for d in data:
            f.write(f'{d:0.2f} ')
        f.write('\n')

def save_objs(objs):
    with open('data/objs.txt', 'w') as f:
        for obj in objs:
            f.write(f'[{obj["start"]}, {obj["end"]}] Size: {obj["size"]} Distance: {obj["distance"]}\t')
            for data in obj['data']:
                f.write(f'{data:0.2f} ')
            f.write('\n')

def save_beacons(beacons):
    with open('data/beacons.txt', 'w') as f:
        for name, value in beacons.items():
            if 'distance' in value:
                f.write(f'{name}: ({value["x"]:.2f}, {value["y"]:.2f}) Phi: {value["last_phi"]:.2f} Gamma: {value["last_gamma"]:.2f} Distance: {value["distance"]:.2f}\n')

def debug_save_data(data):
    with open('data/original.txt', 'w') as f:
        f.write('\n'.join([str(d) for d in data['lidar']]))
    with open('data/new.txt', 'w') as f:
        f.write('\n'.join([str(d) for d in data['lidar_new']]))

#################### MAIN ####################

def print_error(trilateration, ground_truth):
    error_x = trilateration['x'] - ground_truth['x']
    error_y = trilateration['y'] - ground_truth['y']
    error_theta = trilateration['theta'] - ground_truth['theta']
    print(f'Error: ({error_x:.2f}, {error_y:.2f}, {error_theta:.2f})')

def main():
    trilateration = get_initial_trilateration()
    
    while input_data := get_data():
        raw_lidar = input_data['raw_lidar']
    
        trilateration = get_trilateration(raw_lidar, last_trilateration=trilateration)
        
        # print_trilateration(trilateration)
        print_error(trilateration, input_data['ground_truth'])
        save_trilateration(trilateration, ground_truth=input_data['ground_truth'])

        if BREAKPOINT: breakpoint()

def get_args():
    import argparse
    parser = argparse.ArgumentParser(description='Trilateration')
    # get three arguments: x y theta
    parser.add_argument('x', type=float, help='X position')
    parser.add_argument('y', type=float, help='Y position')
    parser.add_argument('theta', type=float, help='Theta position')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--breakpoint', action='store_true', help='Breakpoint')
    return parser.parse_args()

def set_constants(args):
    global INITIAL_AGV_X, INITIAL_AGV_Y, INITIAL_AGV_ANGLE, DEBUG, BREAKPOINT
    INITIAL_AGV_X = args.x
    INITIAL_AGV_Y = args.y
    INITIAL_AGV_ANGLE = args.theta

    DEBUG = args.debug
    BREAKPOINT = args.breakpoint

if __name__ == '__main__':
    # args = get_args()
    # set_constants(args)

    main()

    # trilateration = get_initial_trilateration()

    # beacons = init_beacons(trilateration)
    # for beacon in beacons.values():
    #     beacon['theta'] = beacon['last_gamma']
    #     beacon['distance'] = math.sqrt((beacon['x']-trilateration['x'])**2 + (beacon['y']-trilateration['y'])**2)
    # print_beacons(beacons)

    # x,y = calc_position(beacons['beacon3'], beacons['beacon1'], beacons['beacon2'])
    # print(x,y)

