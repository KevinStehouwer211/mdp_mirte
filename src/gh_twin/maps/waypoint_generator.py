import cv2
import yaml
import numpy as np

# Projected area of the flower box in m^2
box_area = 1.0*0.3
box_area_ub = 1.5*box_area
box_area_lb = 0.5*box_area

image_scale = 5

position_tolerance = 0.5
deadband = 0.2
waypoint_offset = 0.5
N_waypoints = 5 # Number of waypoints along the longer edge of the box


def get_longer_edge_offsets(p1, p2, p3, p4):
    
    waypoint_list =  []
    edge1_point_list = []
    edge2_point_list = []

    # Convert inputs to NumPy vectors
    P1, P2, P3, P4 = map(np.array, [p1, p2, p3, p4])
    
    # 1. Calculate adjacent edge lengths
    L12 = np.linalg.norm(P2 - P1)
    L23 = np.linalg.norm(P3 - P2)
    
    # 2. Identify the two longer parallel edges
    if L12 >= L23:
        # Long edges are P1->P2 and P4->P3
        edge1_start, edge1_end = P1, P2
        edge2_start, edge2_end = P4, P3
    else:
        # Long edges are P2->P3 and P1->P4
        edge1_start, edge1_end = P2, P3
        edge2_start, edge2_end = P1, P4
        
    # 3. Compute unit normal vector for Edge 1
    v1 = edge1_end - edge1_start
    u1 = v1 / np.linalg.norm(v1)
    n1 = np.array([-u1[1], u1[0]]) # Perpendicular normal (-y, x)
    
    edge1_start_offset = edge1_start + deadband*u1/resolution
    edge1_end_offset = edge1_end - deadband*u1/resolution
    
    edge1_point_listx = np.linspace(edge1_start_offset[0], edge1_end_offset[0], N_waypoints)
    edge1_point_listy = np.linspace(edge1_start_offset[1], edge1_end_offset[1], N_waypoints)
    
    for i in range(N_waypoints):
        edge1_point_list.append([edge1_point_listx[i], edge1_point_listy[i]])
    
    # 4. Compute unit normal vector for Edge 2
    v2 = edge2_end - edge2_start
    u2 = v2 / np.linalg.norm(v2)
    n2 = np.array([-u2[1], u2[0]]) # Perpendicular normal (-y, x)
    
    edge2_start_offset = edge2_start + deadband*u2/resolution
    edge2_end_offset = edge2_end - deadband*u2/resolution

    edge2_point_listx = np.linspace(edge2_start_offset[0], edge2_end_offset[0], N_waypoints)
    edge2_point_listy = np.linspace(edge2_start_offset[1], edge2_end_offset[1], N_waypoints)
    
    for i in range(N_waypoints):
        edge2_point_list.append([edge2_point_listx[i], edge2_point_listy[i]])
    

    for point in edge1_point_list:
        
        waypoint = {'x': point[0] - waypoint_offset*n1[0]/resolution, 'y': point[1] - waypoint_offset*n1[1]/resolution, 'yaw': None}
        waypoint_list.append(waypoint)
        
    for point in edge2_point_list:
        waypoint = {'x': point[0] + waypoint_offset*n2[0]/resolution, 'y': point[1] + waypoint_offset*n2[1]/resolution, 'yaw': None}
        waypoint_list.append(waypoint)
        
    return waypoint_list


def coordinate_transform(x, y, map_origin):
    return [x - map_origin[0], -y + map_origin[1]]

def transform_to_real(x,y):
    pass

# 1. Parse YAML Metadata
with open('map.yaml', 'r') as f:
    metadata = yaml.safe_load(f)

resolution = metadata['resolution']
origin_x, origin_y, _ = metadata['origin']

#print(resolution, origin_x, origin_y)

# Read map image
image = cv2.imread('map.pgm', cv2.IMREAD_UNCHANGED)
image_height, image_width = image.shape
map_origin = [int(-(origin_x/resolution)), int((image_height + origin_y/resolution))]

scaled_image = cv2.resize(image, None, fx=image_scale, fy=image_scale, interpolation=cv2.INTER_CUBIC)
white_image = np.full((int(image_height*image_scale), int(image_width*image_scale)), 255, dtype=np.uint8)

# detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
contours, _ = cv2.findContours(image=image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

box_contours = []
box_coordinates = []
waypoints = []
box_id = 0
waypoints_to_export = []

bins = []
waypoints = []



for contour in contours:
    contour_check = False
    area = cv2.contourArea(contour) * (resolution**2) 
    if box_area_lb <= area <= box_area_ub:
        
        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h), angle = rect
     
        for contour_ in box_contours:
            
            
            
            rect_ = cv2.minAreaRect(contour_)
            (cx_, cy_), (w_, h_), angle_ = rect_
            
            if abs(cx - cx_) < (position_tolerance)/resolution and abs(cy - cy_) < (position_tolerance)/resolution:
                contour_check = True
            
        if not contour_check:
            box_contours.append(contour)
            box_points = cv2.boxPoints(rect)
            cv2.drawContours(white_image, [box_points.astype(int)*image_scale], 0, 0, thickness=2)
            cv2.putText(white_image, f"bin_{box_id}", (int(cx*image_scale) - 20, int(cy*image_scale) + 5),  cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
            
            
            contour_check = False
            #box_coordinates.append([cx, cy, w, h])  # Center coordinates and dimensions of the bounding box
            waypoint_bin = get_longer_edge_offsets(box_points[0], box_points[1], box_points[2], box_points[3])
            
            bin = {'id': box_id, 'p1': box_points[0], 'p2': box_points[1], 'p3': box_points[2], 'p4': box_points[3]}
            bins.append(bin)
            
            waypoint_id = 0
            
            
            for point in waypoint_bin:
                x_map, y_map = coordinate_transform(point['x'], point['y'], map_origin)
                wp_id = f"wp_{waypoint_id+2*N_waypoints*box_id}"
                bin_str = f"bin_{box_id}"
                wp_data = {
                    'id': wp_id,
                    'bin_id': bin_str,
                    'x': round(float(x_map*resolution),3),
                    'y': round(float(y_map*resolution),3),
                    'yaw': 0.0,
                    'pose_source': f"manual_slam"
                }
                #waypoint = {'bin_id': box_id, 'id': wp_id, 'x': float(point['x']), 'y': float(point['y']), 'yaw': 0.0, 'pose_source': "SLAM"}
                waypoints_to_export.append(wp_data)
                cv2.circle(white_image, (int(point['x']*image_scale), int(point['y']*image_scale)), radius=2, color=0, thickness=-1)   
                waypoint_id += 1
            
            box_id += 1         

file_path = "waypoints.yaml"

with open(file_path, 'w') as yaml_file:
    # Use standard default_flow_style=False to prevent inline structures
    # Disable sort_keys to retain your exact insertion order
    yaml.dump(waypoints_to_export, yaml_file, default_flow_style=False, sort_keys=False)

cv2.circle(white_image, (int(map_origin[0]*image_scale), int(map_origin[1]*image_scale)), radius=10, color=0, thickness=-1)
cv2.circle(white_image, (0, 0), radius=10, color=0, thickness=-1)

#cropped_roi = white_image[211*image_scale:190*image_scale, 309*image_scale:456*image_scale]
#cropped_roi = white_image[190*image_scale:456*image_scale, 211*image_scale:309*image_scale]

#cv2.rectangle(white_image, (211*image_scale, 190*image_scale), (309*image_scale, 456*image_scale), color=0, thickness=1 )

print(map_origin)
#cv2.drawContours(image=white_image, contours=box_contours, contourIdx=-1, color=(0, 255, 0), thickness=4, lineType=cv2.LINE_AA)
#print(bins)
print(waypoints)
# see the results
cv2.imshow('Extracted contours', white_image)


# Verify the shape and data type
#print("Image dimensions:", image.shape)
#print("Data type:", image.dtype)

#cv2.imshow("Map Image", scaled_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
