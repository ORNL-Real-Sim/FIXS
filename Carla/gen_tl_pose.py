import pandas as pd
import xml.etree.ElementTree as ET
import math
import numpy as np
import csv
from sumo2carla import get_carla_transform

xml_file = 'Roosevelt.net.xml'

tree = ET.parse(xml_file)
root = tree.getroot()
tl_loc = []
carla_tl_loc = []

junc_edges = {'373' : ['-1871', '-1880', '-1868', '-2001']} 

loc = root.find('location')
offset = [float(l) for l in loc.get('netOffset').split(',')]

new_dict = {}

for jn in junc_edges:

    new_dict[jn] = []

    for j in junc_edges[jn]:
        for c in root.findall('connection'):
            if c.attrib['from'] == str(j) and c.attrib['dir'] == 's':
                entry = {"from": c.attrib['from'], "to": c.attrib['to']}
                if entry not in new_dict[jn]:
                    new_dict[jn].append(entry)

    for child in root:

        if child.tag == 'edge':
            # go through the edges that are at junctions and are inc_lanes parents for ex [-116_0] is -116 child
            edge_id = child.attrib['id']
            if edge_id in junc_edges[jn]:

                fr, to = child.attrib['from'], child.attrib['to']
                nxt_jn = [n for n in root.findall('junction') if n.attrib['id'] == str(to)] 
                nxt_jn = nxt_jn[0].attrib
                jn_coords = get_carla_transform(offset=offset, 
                                in_location= [float(nxt_jn['x']), float(nxt_jn['y']), float(nxt_jn['z'])],
                                in_rotation=(0,0,0))

                for n in new_dict[jn]:
                    if n['from'] == edge_id:
                        to_edge = n['to']
                        for n in root.findall('edge'):
                            if n.attrib['id'] == to_edge:
                                lane = n.attrib['shape']
                                to_shape_start = [float(c) for c in lane.split(' ')[0].split(',')]
                                to_shape_end = [float(c) for c in lane.split(' ')[-1].split(',')]

                for l in child:
                    lane = l.attrib['shape']
                    if lane: 
                        coords = lane.split(' ')
                        shape_start =[float(c) for c in coords[0].split(',')]  
                        shape_end = [float(c) for c in coords[-1].split(',')]

                        dist_start = math.sqrt( (float(nxt_jn['x']) - shape_start[0])**2 + (float(nxt_jn['y']) - shape_start[1])**2 )
                        dist_end = math.sqrt( (float(nxt_jn['x']) - shape_end[0])**2 + (float(nxt_jn['y']) - shape_end[1])**2 )
                        dists = [dist_start, dist_end]
                        min_idx = (dists.index(min(dists)))

                        # fyi, in rotation[2] has now been changed to be yaw instead of roll initially it was pitch,yaw,roll as output
                        # now it is roll, pitch, yaw. The traffic light asset faces you at yaw = 0. So if you need the light to face the 
                        # end of the lane, it should be oriented the same way. 
                        if min_idx == 0:
                            # This means that the shape_start is closer
                            bearing = math.degrees( math.atan2( (shape_start[1] - shape_end[1]), (shape_start[0] - shape_end[0]) ) )
                            shape_delta = [to_shape_start[0] - shape_start[0], to_shape_start[1] - shape_start[1]]
                            carla_coords = get_carla_transform(offset=offset, 
                                                            in_location= [shape_start[0] + shape_delta[0], shape_start[1], shape_start[2]],
                                                            in_rotation=(0,0, round(-bearing)) )
                            tl_loc.append([shape_start[0], shape_start[1], l.attrib['id'], jn])
                            carla_tl_loc.append( [carla_coords[0],
                                                carla_coords[1],
                                                carla_coords[2],
                                                carla_coords[3],
                                                carla_coords[4],
                                                carla_coords[5],
                                                jn_coords[0],
                                                jn_coords[1],
                                                jn_coords[2],
                                                l.attrib['id'],
                                                jn] )

                        elif min_idx == 1:
                            # This means that the shape_end is closer
                            bearing = math.degrees( math.atan2( (shape_end[1] - shape_start[1]), (shape_end[0] - shape_start[0]) ) )
                            
                            shape_delta = [to_shape_start[0] - shape_end[0], to_shape_start[1] - shape_end[1]]
                            
                            if abs(bearing - 90) < 5 or abs(bearing + 90) < 5:
                                carla_coords = get_carla_transform(offset=offset, 
                                                                in_location= [shape_end[0], shape_end[1] + shape_delta[1], shape_end[2]],
                                                                in_rotation=(0,0, round(-bearing)) )
                            else:
                                carla_coords = get_carla_transform(offset=offset, 
                                                                in_location= [shape_end[0] + shape_delta[0], shape_end[1], shape_end[2]],
                                                                in_rotation=(0,0, round(-bearing)) )
                                
                            tl_loc.append([shape_end[0], shape_end[1], l.attrib['id'], jn])
                            carla_tl_loc.append( [carla_coords[0],
                                                carla_coords[1],
                                                carla_coords[2],
                                                carla_coords[3],
                                                carla_coords[4],
                                                carla_coords[5],
                                                jn_coords[0],
                                                jn_coords[1],
                                                jn_coords[2], 
                                                l.attrib['id'],
                                                jn] )
                        else:
                            None
                    else:
                        print('No internal lanes found')
        else:
            pass 

tl_loc = np.array(tl_loc)
carla_tl_loc = np.array(carla_tl_loc)

with open('tl_locs.csv','w') as file:
    writer = csv.writer(file)
    writer.writerows(tl_loc)

with open('carla_tl_locs.csv','w') as file1:
    writer = csv.writer(file1)
    writer.writerows(carla_tl_loc)