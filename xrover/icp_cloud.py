import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import struct
import copy
import threading
import time
import std_msgs.msg
from sensor_msgs.msg import PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import itertools


class ICP_pointcloud(Node):
    """ Class này dùng để tạo 1 bản đồ nhỏ từ các pointcloud nhận được - dùng bản đồ nhỏ này để map với tọa độ robot
    Input : pointcloud - nhận từ topic ./custum_cloud
    Output : pointcloud map  - xuất bản lên topic map """
    def __init__(self):
        super().__init__('map_process')
        self.global_cloud = None
        self.threshold = 0.02
        self.trans_init = np.identity(4)
        self._msg = PointCloud2()
        self.old_cloud = None
        self.trans_list = []
        self.pointcloud_list = []
        self.declare_parameter('topic', '/custum_cloud')
        self.map = o3d.geometry.PointCloud()
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(PointCloud2, 'map', 1)
        # Subscribe to the PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.callback,
            1
        )
        self.get_logger().info(f"Subscribed to topic: {topic}")
        # self.timer = self.create_timer(1, self.timer_callback)
        self.isFist = True
    def decode_rgb(self, rgb_float):
        """Chuyển đổi giá trị float32 RGB thành tuple (r, g, b)"""
        rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]  # Chuyển float -> int
        r = ((rgb_int >> 16) & 0xFF) / 255.0
        g = ((rgb_int >> 8) & 0xFF) / 255.0
        b = (rgb_int & 0xFF) / 255.0
        return (r, g, b)
    def callback(self, msg):
        start = time.time()
        points = [] 
        colors = []
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z, rgb = point

            r, g, b = self.decode_rgb(rgb)
            
            points.append((x, y, z))
            colors.append((r, g, b))
        
        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        if len(points) == 0:
            self.get_logger().info("No points received.")
            return  

        mask = (
            (np.abs(points[:, 0]) <= 2) & 
            (np.abs(points[:, 1]) <= 2) & 
            (np.abs(points[:, 2]) <= 2)
        )
        filtered_points = points[mask]
        filtered_colors = colors[mask]
        if filtered_points.shape[0] == 0:
            self.get_logger().info("No valid points after filtering.")
            return  
        
        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
        filtered_cloud = filtered_cloud.voxel_down_sample(voxel_size=0.04)

        # Lọc outlier 
        cl, ind = filtered_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
        filtered_cloud = filtered_cloud.select_by_index(ind)
        voxel_size = 0.04
        
        if self.global_cloud is None:
            source, target, source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(
                voxel_size ,filtered_cloud,filtered_cloud)
            self.global_cloud = filtered_cloud
 
        else:
            source, target, source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(
                voxel_size, self.global_cloud, filtered_cloud)
            result_ransac = self.execute_fast_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
            # print(result_ransac)

            # color registration
            voxel_radius = [0.04, 0.02]
            max_iter = [50, 30]

            for scale in range(2):
                iter = max_iter[scale]
                radius = voxel_radius[scale]

                source_down = source.voxel_down_sample(radius)
                target_down = target.voxel_down_sample(radius)

                source_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                target_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

                result_icp = o3d.pipelines.registration.registration_colored_icp(
                    source_down, target_down, radius, result_ransac.transformation,
                    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                    relative_rmse=1e-6,
                                                                    max_iteration=iter))
                result_ransac.transformation = result_icp.transformation
                # print(result_icp)
            # ngưỡng fitness
            if result_ransac.fitness > 0.11:
                self.draw_registration_result(source_down, target_down, result_icp.transformation)

            else:
                print(":: Not fit icp mapping !")
            print("fitness: ", result_ransac.fitness)
            print("Reading pointcloud data took %.3f sec." % (time.time() - start))

    def encode_rgb(self, r, g, b):
        """Chuyển giá trị màu (0-1) thành uint32_t để lưu trong PointCloud2"""
        r, g, b = int(r * 255), int(g * 255), int(b * 255)
        rgb_int = (r << 16) | (g << 8) | b  
        return rgb_int  

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.transform(transformation)
 
        if self.isFist:
            self.global_cloud.clear()
            self.isFist = False

        self.pointcloud_list.append(source_temp)
        self.global_cloud = target_temp + source_temp
 
        self.global_cloud = self.global_cloud.voxel_down_sample(voxel_size=0.04)
        cl, ind = self.global_cloud.remove_statistical_outlier(nb_neighbors=5, std_ratio=1.5)
        self.global_cloud = self.global_cloud.select_by_index(ind)
        # lọc các điểm nằm ngoài bản đồ
        if self.global_cloud.has_colors():
            colors = np.array(self.global_cloud.colors, dtype=np.float32)
        points = np.array(self.global_cloud.points, dtype=np.float32)
        mask = (
            (np.abs(points[:, 0]) <= 2) & 
            (np.abs(points[:, 1]) <= 2) & 
            (np.abs(points[:, 2]) <= 2)
        )
        filtered_p= points[mask]
        filtered_cl = colors[mask]
        self.global_cloud = o3d.geometry.PointCloud()
        self.global_cloud.points = o3d.utility.Vector3dVector(filtered_p)
        self.global_cloud.colors = o3d.utility.Vector3dVector(filtered_cl)
        if not self.global_cloud.has_colors():
            self.global_cloud.colors = o3d.utility.Vector3dVector(np.random.rand(len(self.global_cloud.points), 3))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
 
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
 
        points_with_color = []
        for (x, y, z), (r, g, b) in zip(np.asarray(self.global_cloud.points), np.asarray(self.global_cloud.colors)):
            rgb = self.encode_rgb(r, g, b) 
            points_with_color.append([x, y, z, rgb])
 
        _msg = pc2.create_cloud(header, fields, points_with_color)
 
        self.publisher_.publish(_msg)
        self.get_logger().info(f"Published map with {len(self.global_cloud.points)} points and color.")

    def preprocess_point_cloud(self, pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)
        radius_normal = voxel_size * 2
        # print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        # print(":: RANSAC registration on downsampled point clouds.")
        # print("   Since the downsampling voxel size is %.3f," % voxel_size)
        # print("   we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000, 0.999))
        return result
    def execute_fast_global_registration(self, source_down, target_down, source_fpfh,
                                        target_fpfh, voxel_size):
        distance_threshold = voxel_size * 0.5
        # print(":: Apply fast global registration with distance threshold %.3f" \
        #         % distance_threshold)
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result

    def prepare_dataset(self, voxel_size, source, target):
        trans_init = np.asarray([[0.0, 0.0, -1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        # trans_init = np.eye(4)
        trans_init_2 = self.rotation_matrix_3d(0, -48, 0)
        target.transform(trans_init)
        target.transform(trans_init_2)
        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        return source, target, source_down, target_down, source_fpfh, target_fpfh
    
    def rotation_matrix_3d(self, angle_x, angle_y, angle_z):
        """Tạo ma trận quay 3D từ 3 góc xoay (đơn vị radian)"""
        ax, ay, az = np.radians(angle_x), np.radians(angle_y), np.radians(angle_z)
        Rx = np.array([[1, 0, 0, 0],
                    [0, np.cos(ax), -np.sin(ax), 0],
                    [0, np.sin(ax), np.cos(ax), 0],
                    [0, 0, 0, 1]])

        Ry = np.array([[np.cos(ay), 0, np.sin(ay), 0],
                    [0, 1, 0, 0],
                    [-np.sin(ay), 0, np.cos(ay), 0],
                    [0, 0, 0, 1]])

        Rz = np.array([[np.cos(az), -np.sin(az), 0, 0],
                    [np.sin(az), np.cos(az), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        return Rz @ Ry @ Rx

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = ICP_pointcloud()
    try:
        while rclpy.ok():
            rclpy.spin_once(pointcloud_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_subscriber.destroy_node()   
        rclpy.shutdown()
       
if __name__ == '__main__':
    main()
