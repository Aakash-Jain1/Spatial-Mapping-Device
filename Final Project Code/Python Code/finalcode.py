import serial
import numpy as np
import open3d as o3d
import math

# Define the angle sequence (in degrees) we are traveling clockwise for data collection
angles_deg = [180, 168.75, 157.5, 146.25, 135, 123.75, 112.5, 101.25, 90, 78.75, 67.5, 56.25, 45, 33.75,
             22.5, 11.25, 0, -11.25, -22.5, -33.75, -45, -56.25, -67.5, -78.75, -90, -101.25, -112.5, -123.75, -135, -146.25, -157.5, -168.75]
# Define the Z-axis positions in millimeters
# 15 steps max displacement set maanually
z_values = [0, 1000,2000,3000,4000,5000,6000,7000,8000, 9000, 10000, 11000 ,12000, 13000, 14000,15000]
# Initialize indices for tracking angles and Z valuesS
j = 0 # Index for Z values
k = 0  # Index for angles
i = 1 # Counter for managing angle and Z iteration

# Serial setup
s = serial.Serial('COM3', 115200, timeout=10)
print("Opening: " + s.name)

# Clear serial buffers before starting
s.reset_output_buffer()
s.reset_input_buffer()

# Open file for writing to store 3D point cloud data
with open("tof_radar.xyz", "w") as f: 
    # Signal MCU to start transmission by pressing enter key
    input("Press Enter to start communication...")

    # Send start command to the MCU
    s.write('s'.encode())

    
    print(s.readline()); # 2dx final code
    print(s.readline()); # model id byte
    print(s.readline()); # module type byte
    print(s.readline()); # module type word
    print(s.readline()); # Model_ID, Module_Type word
    print(s.readline()); # ToF Chip Booted! Please Wait...
    print(s.readline()); # SensorInit status check writing config
    print(s.readline()) ;# SensorInit print check for data ready
    print(s.readline()) ;# status check successful or not   

    

    count = 96 # Total number of readings to process
    processed_count= 0 # Counter for processed data points
    while processed_count<count: # make sure count points are collected and not skipped
        try:
            raw_data = s.readline().decode().strip()  # E.g., "1, 400, 11225, 0, 2"
            print(f"Raw data (string): {raw_data}")  # Debuging

  

            # Split into parts and have commas
            parts = raw_data.split(',')  # Now list like: ["1", "400", "11225", " 0", " 2"]
            print(f"Split parts: {parts}")  # Debuging

            # Isolate distance (2nd value) and get rid of spaces
            distance = int(parts[1].strip())  # "400" → "400" → 400
            print(f"Distance extracted: {distance}")  # Debuging

            # Convert distance to integer
            if len(parts) >= 2:
                distance = int(parts[1].strip())  # Get 2nd value as distance
            else:
                print(f"Invalid format: {raw_data}") # Can't do operations with strings
                continue

            
            # Convert to Cartesian coordinates
            angle_rad = math.radians(angles_deg[k])
            x = math.cos(angle_rad) * distance
            y = math.sin(angle_rad) * distance
            z = z_values[j]

            # Adjust indices for next iteration
            if (i %(len(angles_deg))) == 0 : # after 32 points len(angel_deg) = 32
                j += 1 # z value to next layer
                k= -1 # Reset angle index after layer complete
            i+=1  # Increment counter for next point
            k +=1 # move to the next angle within a layer

            print(f"z value: {z}")
            
            # Write to file
            f.write(f"{x:.2f} {y:.2f} {z}\n")
            print(f"Processed: Angle={angles_deg[k]}°, Distance={distance} → ({x:.2f}, {y:.2f}, {z})")

            processed_count+=1

        except (ValueError, IndexError) as e:
            print(f"Error processing line: {e}")
            continue

# Read and visualize point cloud
print("\nReading point cloud...")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")
if not pcd.points:
    print("Warning: Point cloud is empty! Check output file.")
else:
    print("Point cloud array:")
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd], window_name="Radar Scan")

    # Assign unique identifiers to each vertex
    yz_slice_vertex = []
    for x in range(0,count):
        yz_slice_vertex.append([x])

    # Define coordinates to connect lines in each yz slice    
    # Define connections between points to form a 3D grid    
    lines = []  
    for x in range(0,count,32):
        lines.append([yz_slice_vertex[x + 0], yz_slice_vertex[x + 1]])
        lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 2]])
        lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 3]])
        lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 4]])
        lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 5]])
        lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 6]])
        lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 7]])
        lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 8]])
        lines.append([yz_slice_vertex[x + 8], yz_slice_vertex[x + 9]])
        lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 10]])
        lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 11]])
        lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 12]])
        lines.append([yz_slice_vertex[x + 12], yz_slice_vertex[x + 13]])
        lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 14]])
        lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 15]])
        lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 16]])
        lines.append([yz_slice_vertex[x + 16], yz_slice_vertex[x + 17]])
        lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 18]])
        lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 19]])
        lines.append([yz_slice_vertex[x + 19], yz_slice_vertex[x + 20]])
        lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 21]])
        lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 22]])
        lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 23]])
        lines.append([yz_slice_vertex[x + 23], yz_slice_vertex[x + 24]])
        lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 25]])
        lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 26]])
        lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 27]])
        lines.append([yz_slice_vertex[x + 27], yz_slice_vertex[x + 28]])
        lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 29]])
        lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 30]])
        lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 31]])
        lines.append([yz_slice_vertex[x+ 31], yz_slice_vertex[x]])
        # lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x]])



    #Define coordinates to connect lines between current and next yz slice
    # Connect slices along the Z-axis so range is count -33 
    for x in range(0,count-33,32):
        lines.append([yz_slice_vertex[x + 0], yz_slice_vertex[x + 32]])
        lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 33]])
        lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 34]])
        lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 35]])
        lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 36]])
        lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 37]])
        lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 38]])
        lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 39]])
        lines.append([yz_slice_vertex[x + 8], yz_slice_vertex[x + 40]])
        lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 41]])
        lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 42]])
        lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 43]])
        lines.append([yz_slice_vertex[x + 12], yz_slice_vertex[x + 44]])
        lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 45]])
        lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 46]])
        lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 47]])
        lines.append([yz_slice_vertex[x + 16], yz_slice_vertex[x + 48]])
        lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 49]])
        lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 50]])
        lines.append([yz_slice_vertex[x + 19], yz_slice_vertex[x + 51]])
        lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 52]])
        lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 53]])
        lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 54]])
        lines.append([yz_slice_vertex[x + 23], yz_slice_vertex[x + 55]])
        lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 56]])
        lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 57]])
        lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 58]])
        lines.append([yz_slice_vertex[x + 27], yz_slice_vertex[x + 59]])
        lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 60]])
        lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 61]])
        lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 62]])
        lines.append([yz_slice_vertex[x + 31], yz_slice_vertex[x + 63]])

    #This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

    #Lets see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
                                    
    
# Close the serial connection
s.close()
print("Done.")