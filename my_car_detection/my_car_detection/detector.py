import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ultralytics import YOLO
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer 
import numpy as np
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, TransformStamped
import yaml
from pathlib import Path
import math
from my_car_interface.msg import NavComplete

class detector_node(Node):
    def __init__(self, nodeName:str):
        super().__init__(nodeName)

        self.fx = None  #camera intrinsics
        self.fy = None
        self.cx = None
        self.cy = None

        self.det_obj = {} #key is cls_name adnd val is arr of objs
        self.log_path = Path.home()/"car1"/"src"/"my_car_detection"/"saved_objects"/"objs_log.yaml"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_converter = CvBridge()
        self.detection_model = YOLO("yolo26n-seg.pt")   #YOLO segmentation model

        self.rgb_subber = Subscriber(self, Image, "camera/image", 5)          
        self.depth_subber = Subscriber(self, Image, "camera/depth_image", 5)
        self.rgb_depth_pair = TimeSynchronizer([self.rgb_subber, self.depth_subber], 5)
        self.rgb_depth_pair.registerCallback(self.detect)

        self.cam_info_subber = self.create_subscription(CameraInfo, "camera/camera_info", self.cam_info, 1)
        self.camera_label_publisher = self.create_publisher(Image, "camera/image_label", 1)

        self.nav_status_subber = self.create_subscription(NavComplete, "nav_status", self.nav_status_sub, 10)
        self.stop_detect = False
 
    #process an image, detect objects, and log them (if non-duplicate)
    def detect(self, rgb_msg: Image, depth_msg: Image):
        bgr_arr = self.image_converter.imgmsg_to_cv2(rgb_msg, "bgr8") 
        results = self.detection_model(bgr_arr, imgsz = 1280, conf=0.8, retina_masks=True)
        self.image_label(results, rgb_msg.header)   

        if self.stop_detect: 
            #if mapping complete, stop detection
            return

        depth_arr = self.image_converter.imgmsg_to_cv2(depth_msg, "passthrough") 
        if depth_arr.shape != bgr_arr.shape[:2]: #depth values and bgr values don't have same dimensions
            return  

        #an image may have many masks. So, predict_result.masks.data may have many elements (masks). 
        #masks for pixels w/i a detected object have value 1. Masks for pixels outside the object have value 0.
        detect_result = results[0]   
        if detect_result.masks == None or len(detect_result.masks.data) == 0:
            return

        #iterate every pixel mask in 'data' array to see which pixel is w/i each detected object (masked 1). 
        #save depth values of pixel that are masked 1 (via bool indexing) and valid.
        for index, pixel_mask in enumerate(detect_result.masks.data):
            pixel_mask_bool = pixel_mask.cpu().numpy().astype(bool) #convert pixel mask to bool np arr
            if pixel_mask_bool.shape != depth_arr.shape:
                continue
            
            valid_depths = depth_arr[pixel_mask_bool]   
            valid_depths = valid_depths[np.isfinite(valid_depths) & (valid_depths > 0)]
            if len(valid_depths) == 0:
                continue

            #find a pixel in the object w/ valid depth closest to median of saved depth values.
            #this pixel represents the object.
            med_depth = np.median(valid_depths)
            depth_mask = pixel_mask_bool & np.isfinite(depth_arr) & (depth_arr > 0)
            rows, cols = np.nonzero(depth_mask) #coord arrs of pixels w/ valid/masked depths. 
            depths = depth_arr[rows, cols]      #depth values of pixels
            target_pixel_ind = np.argmin(np.abs(depths - med_depth))
            p_u, p_v, p_d = cols[target_pixel_ind], rows[target_pixel_ind], depths[target_pixel_ind] 

            p_xyz = self.convert_to_3d_cam(p_u, p_v, p_d) 
            if p_xyz == None:
                return
            p_x, p_y, p_z = p_xyz   #pixel in 3d coords on cam's optical_link
            p_on_cam = self.create_point_stamped(p_x, p_y, p_z, rgb_msg, "optical_link")

            cam_to_map_tf = self.tf_optical_link_to_map(rgb_msg)
            if cam_to_map_tf == None:
                return
            p_on_map = do_transform_point(p_on_cam, cam_to_map_tf) #map coords of pixel representing the object.

            #check if this object is already seen from different angles
            pixel_mask_cls_id = detect_result.boxes.cls[index].item()
            self.object_check(p_on_map, detect_result.names[pixel_mask_cls_id])

            #FIXME: now save a PoseStampedd point near this pixel on map, but with z = 0 and away from the pixel for about 0.2m. 
            #Reason: the pixel can be wall

    #check if the object is already detected, to avoid double-count.
    #if object is new, add it to det_obj dict and log it in objs_log.yaml.
    def object_check(self, obj: PointStamped, cls_name: str):
        if cls_name == "airplane" or cls_name == "traffic light": 
            #filter out noise
            return

        if cls_name not in self.det_obj: #if object is new
            self.det_obj[cls_name] = [obj]
            obj_log = {
                f"{cls_name}_0": {
                    "x": float(obj.point.x),
                    "y": float(obj.point.y),
                    "z": float(obj.point.z)
                }
            }
            with open(self.log_path, "a", encoding="utf-8") as file:
                yaml.safe_dump(obj_log, file, sort_keys=False)
                file.write("\n")
        else:
            for ob in self.det_obj[cls_name]:
                ob: PointStamped
                obj_x = obj.point.x
                obj_y = obj.point.y
                obj_z = obj.point.z
                ob_x = ob.point.x
                ob_y = ob.point.y
                ob_z = ob.point.z
                if math.dist([obj_x, obj_y, obj_z], [ob_x, ob_y, ob_z]) <= 0.3:
                    return
                
            self.det_obj[cls_name].append(obj)
            suffix = len(self.det_obj[cls_name]) - 1
            obj_log = {
                f"{cls_name}_{suffix}": {
                    "x": float(obj.point.x),
                    "y": float(obj.point.y),
                    "z": float(obj.point.z)
                }
            }
            with open(self.log_path, "a", encoding="utf-8") as file:
                yaml.safe_dump(obj_log, file, sort_keys=False)
                file.write("\n")

    def cam_info(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    #convert img coords into 3d coords on camera's optical_link
    def convert_to_3d_cam(self, u: float, v: float, d: float) -> list | None:
        if self.fx == None or self.fy == None or self.fx <= 0 or self.fy <= 0 or self.cx == None or self.cy == None:
            return None
        x = (u - self.cx) * d / self.fx
        y = (v - self.cy) * d / self.fy
        z = d
        coords_3d = [x, y, z]
        return coords_3d

    #retrieve tf from optical_link to map at timestamp of rgb_msg being processed
    def tf_optical_link_to_map(self, rgb_msg: Image) -> TransformStamped | None:
        timestamp = Time.from_msg(rgb_msg.header.stamp) #convert rgb_msg's time to ros2 time
        try:
            return self.tf_buffer.lookup_transform("map", "optical_link", timestamp)
        except TransformException as e:
            return None

    def image_label(self, results, image_header):
        labeled_bgr_arr = results[0].plot() #label image
        labeled_rgb_msg = self.image_converter.cv2_to_imgmsg(labeled_bgr_arr, "bgr8")
        labeled_rgb_msg.header = image_header
        self.camera_label_publisher.publish(labeled_rgb_msg)

    def create_point_stamped(self, x, y, z, rgb_msg, frame_id) -> PointStamped:
        point_stamped = PointStamped()
        point_stamped.header.stamp = rgb_msg.header.stamp
        point_stamped.header.frame_id = frame_id
        point_stamped.point.x = float(x)
        point_stamped.point.y = float(y)
        point_stamped.point.z = float(z)
        return point_stamped

    def nav_status_sub(self, msg: NavComplete):
        if msg.nav_complete:
            self.stop_detect = True

def main(args=None):
    rclpy.init(args=args)
    detector = detector_node("detector_node")

    rclpy.spin(detector)
    rclpy.shutdown()

if __name__ == "__main__":
    main()