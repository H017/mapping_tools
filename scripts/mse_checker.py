import roslib
roslib.load_manifest('mapping_tools')
import rospy
from sensor_msgs.msg import LaserScan
import math
import Image
import ImageDraw


def convert_to_list(scan, angle_offset=0):
    l = set()
    for i in range(len(scan.ranges)):
        if scan.ranges[i] != 0:
            angle = scan.angle_min + i * scan.angle_increment + angle_offset
            x = math.cos(angle) * scan.ranges[i]
            y = math.sin(angle) * scan.ranges[i]
            l.add((x, y))
    return l


def find_min_max(list1, list2):
    minx = 99999999
    maxx = -9999999
    miny = 999999999
    maxy = -99999999

    for i in range(2):
        l = list1 if i == 0 else list2
        for p in l:
            if p[0] < minx:
                minx = p[0]
            if p[0] > maxx:
                maxx = p[0]
            if p[1] < miny:
                miny = p[1]
            if p[1] > maxy:
                maxy = p[1]

    return minx, maxx, miny, maxy


def save_image(l, l2):
    minx, maxx, miny, maxy = find_min_max(l, l2)

    im = Image.new('RGB', (int((maxx-minx)*100), int((maxy-miny)*100)), (0, 0, 0))
    draw = ImageDraw.Draw(im)
    for p in scan:
        point, dist = find_closest_point(l2, p)
        draw.line((int((p[0]-minx)*100), int((p[1]-miny)*100), int((point[0]-minx)*100), int((point[1]-miny)*100)), fill=(0, 0, 125))

    for p in l:
        x = int((p[0]-minx)*100)
        y = int((p[1]-miny)*100)
        draw.ellipse((x-1, y-1, x+1, y+1), fill=(255, 0, 0))
    for p in l2:
        x = int((p[0]-minx)*100)
        y = int((p[1]-miny)*100)
        draw.ellipse((x-1, y-1, x+1, y+1), fill=(0, 128, 0))
    del draw

    im.save("scans.png", "PNG")

def find_closest_point(scan, point):
    min_dist = 9999999
    min_point = None
    for p in scan:
        dist = math.hypot(point[0]-p[0], point[1]-p[1])
        if dist < min_dist:
            min_dist = dist
            min_point = p
    return min_point, min_dist


rospy.init_node('mse_checker', anonymous=True)

scan = convert_to_list(rospy.wait_for_message("/scan", LaserScan, timeout=None))
scan2 = convert_to_list(rospy.wait_for_message("/scan", LaserScan, timeout=None))

save_image(scan, scan2)

total = 0
for p in scan:
    point, dist = find_closest_point(scan2, p)
    total += math.pow(dist, 2)

print "MSE: %f" % (total / len(scan))

