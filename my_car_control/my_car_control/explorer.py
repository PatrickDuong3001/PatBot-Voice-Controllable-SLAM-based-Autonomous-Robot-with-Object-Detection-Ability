import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from my_car_control.frontier_cell import Frontier_Cell
from geometry_msgs.msg import PoseStamped
from queue import Queue
import math
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from nav2_msgs.srv import SaveMap
from pathlib import Path
from datetime import datetime
from my_car_interface.msg import NavComplete

INF = float('inf')

class explorer_node(Node):
    def __init__(self, nodeName:str):
        super().__init__(nodeName)
        self.controller = self.create_subscription(OccupancyGrid, "map", self.control, 10)
        self.path_compute_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")        

        self.pose_retriever = self.create_subscription(TFMessage, "tf", self.pose_retrieve, 10)
        self.map_client = self.create_client(SaveMap,"map_saver/save_map")
        self.status_broadcaster = self.create_publisher(NavComplete, "nav_status", 10)
        self.timer = self.create_timer(1, self.broadcast)
        
        self.nav_msg = NavComplete()
        self.nav_msg.nav_complete = False

        self.current_pose = None
        self.candidate_goal_poses = []
        self.min_path_len = INF
        self.target_pose_index = -1
        self.candidate_index = 0

        self.iteration_with_no_candidate_goal_pose = 0
        self.map_dir = Path.home()/"car1"/"src"/"my_car_bringup"/"maps" #folder to save maps
        self.map_saved = False

        #status of explorer node. ready, plan, or navigate. W/o this, explorer node can keep getting new Occupancy Grid even tho computing path or navigating is not done
        self.status = "ready" 
        self.frontier = set() #frontier cells in occupancy grid coordinates

    def reset(self):
        #reset everything to ready for planning and navigation
        self.candidate_index = 0
        self.min_path_len = INF
        self.candidate_goal_poses = []
        self.frontier = set()
        self.target_pose_index = -1
        self.status = "planning"

    #Retrieve occupancy grid from /map topic.
    #Find frontiers in the occupancy grid by iterating the grid, check if a cell is free, and check if it has any unknown neightbors.
    #If a cell is free and has at least 1 unknown neightbor, then it is a frontier.
    def control(self, msg: OccupancyGrid):
        if self.status != "ready": #if explorer is busy (e.g., planning, navigating), do not retrieve new occupancy grid
            return

        cell_status = msg.data #a cell status array. NOT occupancy grid
        grid_width = msg.info.width    #521 cells
        grid_height = msg.info.height  #460 cells

        #check if the grid is valid, and length of status array matches dimensions of occupancy grid.
        if grid_width == 0 or grid_height == 0 or len(cell_status) != grid_width * grid_height:
            return

        #a frontier cell must be a free cell.
        #if no free cell, then no frontier cell. Skip this iteration
        if 0 not in cell_status:
            return
        
        self.reset()

        grid_res = msg.info.resolution #0.05      
        o_x = msg.info.origin.position.x
        o_y = msg.info.origin.position.y 

        num_col = grid_width
        num_row = grid_height

        #find all the frontier cells
        for row in range(0,num_row):
            for col in range(0, num_col):
                occupancy_status = cell_status[row * grid_width + col] #in cell_status, index i = num_row * grid_width + num_col

                up_status, down_status, left_status, right_status = -100, -100, -100, -100
                if (occupancy_status == 0):
                    if (row > 0): 
                        up_status = cell_status[(row-1) * grid_width + col]
                        if up_status == -1:
                            frontier_cell = Frontier_Cell(col, row, o_x, o_y, grid_res)
                            self.frontier.add(frontier_cell) 
                            continue
                    if (row < num_row - 1):
                        down_status = cell_status[(row+1) * grid_width + col]
                        if down_status == -1:
                            frontier_cell = Frontier_Cell(col, row, o_x, o_y, grid_res)
                            self.frontier.add(frontier_cell)                             
                            continue
                    if (col > 0):
                        left_status = cell_status[row * grid_width + (col-1)]
                        if left_status == -1:
                            frontier_cell = Frontier_Cell(col, row, o_x, o_y, grid_res)
                            self.frontier.add(frontier_cell) 
                            continue
                    if (col < num_col - 1):
                        right_status = cell_status[row * grid_width + (col+1)]
                        if right_status == -1:
                            frontier_cell = Frontier_Cell(col, row, o_x, o_y, grid_res)
                            self.frontier.add(frontier_cell) 
                            continue

        self.candidate_goal_poses = self.find_goal_pose(o_x, o_y, grid_res)

        #now, check if no goal poses are found for 5 iterations. 
        #If so, then the mapping is complete. Wait until we reach 5 iterations. 
        if len(self.candidate_goal_poses) == 0:
            self.iteration_with_no_candidate_goal_pose += 1
            if self.iteration_with_no_candidate_goal_pose == 5 and self.map_saved == False:
                self.map_save()
            else:
                self.status = "ready"
            return
        self.iteration_with_no_candidate_goal_pose = 0  #if there are goal poses b4 reach 5 iterations, reset this variable.
        self.find_navigation_goal()

    #find all paths from the robot to goal poses, and
    #identify the shortest path. 
    def find_navigation_goal(self):
        if len(self.candidate_goal_poses) == 0 or not self.path_compute_client.server_is_ready(): #no candidate goal poses or action server unready
            self.status = "ready" #go back to retrieving occupancy grid
            return
        if self.candidate_index >= len(self.candidate_goal_poses): #all candidate goal poses are already evaluated
            if self.target_pose_index == -1:  #no valid goal is found from the candidates
                self.status = "ready"   #no navigation
            else:
                self.navigate_to_goal() #navigate to the valid goal
            return
        
        candidate_goal_pose = self.candidate_goal_poses[self.candidate_index]
        action_request = ComputePathToPose.Goal()
        action_request.goal = self.create_pose_stamped("map", candidate_goal_pose.get_real_X(), candidate_goal_pose.get_real_Y())
        goal_future = self.path_compute_client.send_goal_async(action_request)
        goal_future.add_done_callback(self.compute_goal_response)

    #verify goal response from nav2 planner
    def compute_goal_response(self, future: Future):
        goal_response = future.result()
        if goal_response.accepted: #check if goal sent by path_compute_client is accepted by nav2 planner.
            result_future = goal_response.get_result_async()
            result_future.add_done_callback(self.compute_result_response)
        else: #if goal is rejected, move onto next candidate goal pose
            self.candidate_index += 1
            self.find_navigation_goal()

    #get path result from nav2 planner
    def compute_result_response(self, future: Future):
        result_response = future.result()
        if result_response.status == GoalStatus.STATUS_SUCCEEDED: #check if planning result succeeded    
            path_of_poses = result_response.result.path.poses
            if len(path_of_poses) != 0:       #and check if the path is non-empty
                path_length = self.compute_path_length(path_of_poses) 
                if path_length < self.min_path_len:
                    self.min_path_len = path_length
                    self.target_pose_index = self.candidate_index
        if result_response.status == ComputePathToPose.Result.START_OCCUPIED: #check if nav2 planner rejects robot pose
            self.status = "ready"
            return
        self.candidate_index += 1 
        self.find_navigation_goal()

    def navigate_to_goal(self):
        if not self.navigate_client.server_is_ready() or self.target_pose_index == -1: #if action server unready or no valid goal pose, back to retrive occupancy grid
            self.status = "ready"
            return
        self.status = "navigating"
        selected_goal_pose = self.candidate_goal_poses[self.target_pose_index]
        action_request = NavigateToPose.Goal()
        action_request.pose = self.create_pose_stamped("map", selected_goal_pose.get_real_X(), selected_goal_pose.get_real_Y())
        goal_future = self.navigate_client.send_goal_async(action_request)
        goal_future.add_done_callback(self.navigate_goal_response) 

    def navigate_goal_response(self, future: Future):
        goal_response = future.result()
        if goal_response.accepted: 
            result_future = goal_response.get_result_async()
            result_future.add_done_callback(self.navigate_result_response)
        else: 
            self.status = "ready"

    def navigate_result_response(self, future: Future):
        result_response = future.result()
        if result_response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation Suceeded")
        else:
            self.get_logger().warning("Navigation Failed")
        self.status = "ready"


    def find_goal_pose(self, o_x, o_y, grid_res):
        cluster_list = []

        #cluster the found frontier cells into clusters using BFS
        while len(self.frontier) != 0:
            visited = set()
            temp = Queue()

            temp.put(self.frontier.pop())

            cluster = []
            while (not temp.empty()):
                curr_frontier = temp.get()

                if curr_frontier in visited:
                    continue

                visited.add(curr_frontier) #immediate mark a frontier cell as visited after dequeuing it
                cluster.append(curr_frontier)
                
                curr_grid_x = curr_frontier.get_grid_x()
                curr_grid_y = curr_frontier.get_grid_y()

                #find all neighboring cells
                right_cell = Frontier_Cell(curr_grid_x + 1, curr_grid_y, o_x, o_y, grid_res)
                left_cell = Frontier_Cell(curr_grid_x - 1, curr_grid_y, o_x, o_y, grid_res)
                up_cell = Frontier_Cell(curr_grid_x, curr_grid_y - 1, o_x, o_y, grid_res)
                down_cell = Frontier_Cell(curr_grid_x, curr_grid_y + 1, o_x, o_y, grid_res)
                up_right_cell = Frontier_Cell(curr_grid_x + 1, curr_grid_y - 1, o_x, o_y, grid_res)
                up_left_cell = Frontier_Cell(curr_grid_x - 1, curr_grid_y - 1, o_x, o_y, grid_res)
                down_right_cell = Frontier_Cell(curr_grid_x + 1, curr_grid_y + 1, o_x, o_y, grid_res)
                down_left_cell = Frontier_Cell(curr_grid_x - 1, curr_grid_y + 1, o_x, o_y, grid_res)

                #search for adjacent frontier cells
                if right_cell in self.frontier and right_cell not in visited:
                    temp.put(right_cell)
                if left_cell in self.frontier and left_cell not in visited:
                    temp.put(left_cell)
                if up_cell in self.frontier and up_cell not in visited:
                    temp.put(up_cell)
                if down_cell in self.frontier and down_cell not in visited:
                    temp.put(down_cell)
                if up_right_cell in self.frontier and up_right_cell not in visited:
                    temp.put(up_right_cell)
                if up_left_cell in self.frontier and up_left_cell not in visited:
                    temp.put(up_left_cell)
                if down_right_cell in self.frontier and down_right_cell not in visited:
                    temp.put(down_right_cell)
                if down_left_cell in self.frontier and down_left_cell not in visited:
                    temp.put(down_left_cell)
            self.frontier.difference_update(cluster) #remove all frontier cells in a cluster from original frontier set

            #if the cluster is too small, don't add it to cluster_list to keep cluster_list small.
            #this is to reduce the number of centroids and number of candidate goal poses we need to compute.
            if len(cluster) >= 15:
                cluster_list.append(cluster)

        #compute a centroid for each cluster. Then, for each cluster, choose a frontier cell closest to the centroid as a candidate goal pose.
        candidate_goal_poses = []
        for cluster in cluster_list:
            centroid_x, centroid_y = self.centroid_calculate(cluster)
            min_dist = INF
            target_index = -1
            for index, cell in enumerate(cluster):
                cell: Frontier_Cell
                dist = math.sqrt((centroid_x - cell.get_grid_x())*(centroid_x - cell.get_grid_x()) + (centroid_y - cell.get_grid_y())*(centroid_y - cell.get_grid_y()))
                if dist < min_dist:
                    min_dist = dist
                    target_index = index
            candidate_goal_poses.append(cluster[target_index])
        return candidate_goal_poses

    def centroid_calculate(self, cluster):
        sum_x = 0.0
        sum_y = 0.0
        num_frontier_cells = len(cluster)
        for frontier_cell in cluster: 
            frontier_cell: Frontier_Cell
            sum_x += frontier_cell.get_grid_x()
            sum_y += frontier_cell.get_grid_y()
        return sum_x / num_frontier_cells, sum_y / num_frontier_cells

    #compute distance between a candidate goal pose and the current robot's pose
    def compute_path_length(self, poses: list[PoseStamped]) -> float:
        total_dist = 0 
        l = len(poses)
        for i in range(0, l - 1):
            x_1 = poses[i].pose.position.x
            y_1 = poses[i].pose.position.y
            x_2 = poses[i + 1].pose.position.x 
            y_2 = poses[i + 1].pose.position.y
            total_dist += math.sqrt((x_2 - x_1)*(x_2 - x_1) + (y_2 - y_1)*(y_2 - y_1))
        return total_dist

    def pose_retrieve(self, msg: TFMessage):
        self.current_pose = msg

    def create_pose_stamped(self, frame_id: str, p_x: float, p_y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = p_x
        pose.pose.position.y = p_y

        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(0, 0, 0) 
        return pose

    def map_save(self):
        save_request = SaveMap.Request()
        save_request.map_topic = "/map"
        save_request.map_url = str(self.map_dir/f"map_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}")
        save_request.image_format = "pgm"
        save_request.map_mode = "trinary"
        save_request.free_thresh = 0.25
        save_request.occupied_thresh = 0.65
        
        self.map_client.call_async(save_request)
        self.map_saved = True
        self.status = "complete"
        self.nav_msg.nav_complete = True

    def broadcast(self): #publish mapping status
        self.status_broadcaster.publish(self.nav_msg)

def main(args=None):
    rclpy.init(args=args)
    explorer = explorer_node("explorer_node")

    rclpy.spin(explorer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()