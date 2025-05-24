import rospy
from nav_msgs.msg import OccupancyGrid
import math

class CostmapChecker:
    def __init__(self):
        self.costmap = None
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_cb)

    def costmap_cb(self, msg):
        self.costmap = msg

    def is_pose_safe(self, x, y, robot_radius=0.3, cost_threshold=50):
        """
        Kiểm tra vùng quanh (x,y) trên costmap có an toàn không.

        robot_radius: bán kính robot tính theo số ô.  
        cost_threshold: ngưỡng giá trị cost để coi là chướng ngại.

        Trả về True nếu vùng an toàn, False nếu có vật cản.
        """

        if self.costmap is None:
            rospy.logwarn("Chưa nhận được costmap")
            return False

        info = self.costmap.info
        data = self.costmap.data

        # Tính ô trong grid từ tọa độ x,y (theo frame map)
        map_x = int((x - info.origin.position.x) / info.resolution)
        map_y = int((y - info.origin.position.y) / info.resolution)

        # Số ô bán kính robot
        radius_cells = int(robot_radius / info.resolution)

        width = info.width
        height = info.height

        # Kiểm tra ô trong vòng bán kính robot
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                cx = map_x + dx
                cy = map_y + dy

                if cx < 0 or cx >= width or cy < 0 or cy >= height:
                    # Ngoài bản đồ -> coi là nguy hiểm
                    return False

                idx = cy * width + cx
                if data[idx] >= cost_threshold:
                    return False  # Ô có vật cản

        return True  # Vùng an toàn
