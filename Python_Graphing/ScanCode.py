import serial
import math
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Serial connection settings
SERIAL_PORT = "COM6"      
BAUD_RATE   = 115200      
TIMEOUT_SEC = 1           

# Output file names
XYZ_FILENAME = "scan_points.xyz"
CSV_FILENAME = "scan_points.csv"


# Returns true if the scan number should be treated as counterclockwise
# Even numbered scans are CCW odd-numbered scans are CW
def is_ccw(scan_num):
    return scan_num % 2 == 0


# Stores every valid point from all scans
all_points = []

scans = {}

current_scan = 0
current_x_mm = 0


# Converts scanner data into 3D Cartesian coordinates
def convert_to_xyz(x_mm, angle_mdeg, distance_mm):
    angle_deg = angle_mdeg / 1000.0          # Convert millidegrees to degrees
    theta = math.radians(angle_deg)          # Convert degrees to radians for trig

    x = float(x_mm)
    y = float(distance_mm) * math.cos(theta)
    z = float(distance_mm) * math.sin(theta)

    return x, y, z, angle_deg


# Saves points in XYZ format:
# each line contains x y z
def save_xyz(points, filename):
    with open(filename, "w") as f:
        for p in points:
            f.write(f"{p['x']:.2f} {p['y']:.2f} {p['z']:.2f}\n")


# Saves point data in CSV format data
def save_csv(points, filename):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["scan", "x_mm", "angle_deg", "distance_mm", "x", "y", "z"])

        for p in points:
            writer.writerow([
                p["scan"],
                p["x_mm"],
                p["angle_deg"],
                p["distance_mm"],
                p["x"],
                p["y"],
                p["z"]
            ])


# Parses a line 
def parse_point_line(line):
    try:
        parts = line.split(",")

        # Expected exactly 4 comma-separated parts
        if len(parts) != 4:
            return None

        x_mm = int(parts[1].strip().replace("Offset: ", "").replace(" mm", ""))
        angle_mdeg = int(parts[2].strip().replace("Angle: ", "").replace(" mdeg", ""))
        distance_mm = int(parts[3].strip().replace("Distance: ", "").replace(" mm", ""))

        return x_mm, angle_mdeg, distance_mm

    except (ValueError, IndexError):
        return None


# Creates a 3D wireframe plot of all scans
def plot_wireframe(scan_dict):
    fig = plt.figure(figsize=(12, 9))
    fig.patch.set_facecolor("white")

    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("white")

    # Sort scan numbers so they are plotted in order
    ordered_scan_nums = sorted(scan_dict.keys())

    # Reuse these colors for scans
    scan_colors = ["#1f77b4", "#2ca02c", "#d62728"]

    # Plot each scan as a closed shape
    for idx, scan_num in enumerate(ordered_scan_nums):
        pts = scan_dict[scan_num]
        color = scan_colors[idx % len(scan_colors)]

        if len(pts) == 0:
            continue

        xs = [p["x"] for p in pts]
        ys = [p["y"] for p in pts]
        zs = [p["z"] for p in pts]

        # Close the loop by adding the first point again at the end
        xs_c = xs + [xs[0]]
        ys_c = ys + [ys[0]]
        zs_c = zs + [zs[0]]

        # Draw the perimeter of the scan
        ax.plot(
            xs_c, ys_c, zs_c,
            color=color,
            linewidth=2,
            label=f"Scan {scan_num} (X={pts[0]['x_mm']}mm)"
        )

        # Draw the scan points
        ax.scatter(xs, ys, zs, color=color, s=15, zorder=5)

    # Connect corresponding points between adjacent scans
    for i in range(len(ordered_scan_nums) - 1):
        scan_a = scan_dict[ordered_scan_nums[i]]
        scan_b = scan_dict[ordered_scan_nums[i + 1]]

        # If the second scan was taken in CCW order reverse it so point-to-point connections line up properly with the previous scan
        if is_ccw(ordered_scan_nums[i + 1]):
            scan_b = list(reversed(scan_b))

        # Only connect up to the smaller number of points
        count = min(len(scan_a), len(scan_b))

        for j in range(count):
            ax.plot(
                [scan_a[j]["x"], scan_b[j]["x"]],
                [scan_a[j]["y"], scan_b[j]["y"]],
                [scan_a[j]["z"], scan_b[j]["z"]],
                color="black",
                linewidth=0.8,
                alpha=0.8
            )

    # Label the plot
    ax.set_title("2DX3 Spatial Mapping - 3D Wireframe", fontsize=13, fontweight="bold")
    ax.set_xlabel("X Depth (mm)", fontsize=10)
    ax.set_ylabel("Y Width (mm)", fontsize=10)
    ax.set_zlabel("Z Height (mm)", fontsize=10)

    ax.legend(fontsize=9)
    ax.view_init(elev=25, azim=-60)  # Set the 3D viewing angle

    plt.tight_layout()
    plt.savefig("scan_3d.png", dpi=150)
    print("Plot saved as scan_3d.png")
    plt.show()


# Open the serial port
print(f"Opening serial port: {SERIAL_PORT} at {BAUD_RATE} baud")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC)

# Clear any leftover data in the buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

print("Listening for scan data...")
print("Press PJ0 on the board to start each scan.")
print()


try:
    while True:
        # Read one line from the serial port
        raw_line = ser.readline()

        # If nothing was received keep waiting
        if not raw_line:
            continue

        # Decode bytes into a string and strip whitespace
        line = raw_line.decode(errors="ignore").strip()

        if line == "":
            continue

        print(line)

        # Stop when the board indicates scanning is complete
        if line == "All scans complete":
            print("\nFinished receiving all scans.")
            break

        # Update the current scan number when a SCAN_INDEX line is received
        if line.startswith("SCAN_INDEX,"):
            try:
                current_scan = int(line.split(",")[1])
                print(f"  -> Now receiving scan {current_scan}")
            except ValueError:
                pass

        # Update the current X offset when an X_MM line is received
        if line.startswith("X_MM,"):
            try:
                current_x_mm = int(line.split(",")[1])
            except ValueError:
                pass

        # Parse point measurement lines
        if line.startswith("Point "):
            result = parse_point_line(line)

            # Skip malformed point lines
            if result is None:
                print(f"  -> Skipping invalid line: {line}")
                continue

            x_mm, angle_mdeg, distance_mm = result

            # Ignore zero distance readings
            if distance_mm == 0:
                print(f"  -> Skipping zero distance at angle {angle_mdeg} mdeg")
                continue

            # Convert the point to 3D coordinates
            x, y, z, angle_deg = convert_to_xyz(x_mm, angle_mdeg, distance_mm)

            # Store all useful data in a dictionary
            point = {
                "scan": current_scan,
                "x_mm": x_mm,
                "angle_deg": angle_deg,
                "distance_mm": distance_mm,
                "x": x,
                "y": y,
                "z": z
            }

            # Add to master point list
            all_points.append(point)

            # Add to the correct scan group
            if current_scan not in scans:
                scans[current_scan] = []

            scans[current_scan].append(point)

finally:
    # Close the serial port even if an error occurs
    ser.close()
    print("Serial port closed.")


# If no data collected then error message
if len(all_points) == 0:
    print("No scan data received. Check COM port and board connection.")
else:
    # Saves data
    save_xyz(all_points, XYZ_FILENAME)
    save_csv(all_points, CSV_FILENAME)

    print(f"\nSaved {len(all_points)} points to {XYZ_FILENAME} and {CSV_FILENAME}")
    print()

    # Print how many valid points each scan contains
    for s in sorted(scans.keys()):
        print(f"  Scan {s}: {len(scans[s])} valid points")

    print()

    # Display the 3D wireframe
    plot_wireframe(scans)