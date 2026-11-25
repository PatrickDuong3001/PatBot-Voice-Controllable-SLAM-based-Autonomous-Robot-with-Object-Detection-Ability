import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
from pathlib import Path
from my_car_interface.msg import NavComplete
from geometry_msgs.msg import PoseStamped
import yaml
from rclpy.task import Future
import sounddevice as sd
from queue import Queue, Empty
from playsound3 import playsound
from vosk import Model, KaldiRecognizer
import json
import re

class follower_node(Node):
    def __init__(self, nodeName: str):
        super().__init__(nodeName)
        self.det_obj = {}
        self.log_path = Path.home()/"car1"/"src"/"my_car_detection"/"saved_objects"/"objs_log.yaml"
        self.status = "wait"
        self.audio_queue = Queue()

        self.ready_sound_path = Path.home()/"car1"/"src"/"my_car_sound"/"ready.mp3"
        self.listen_sound_path = Path.home()/"car1"/"src"/"my_car_sound"/"listen.mp3"
        self.go_sound_path = Path.home()/"car1"/"src"/"my_car_sound"/"go.mp3"
        self.fail_sound_path = Path.home()/"car1"/"src"/"my_car_sound"/"fail.mp3"
        self.found_sound_path = Path.home()/"car1"/"src"/"my_car_sound"/"found.mp3"

        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")       
        self.path_compute_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.nav_status_subber = self.create_subscription(NavComplete, "nav_status", self.nav_status_sub, 10)
        self.follow_checker = self.create_subscription(Log, "rosout", self.follow_check, 10)
        self.goal_pose = None

        self.audio_input = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1, callback=self.audio_callback)
        self.audio_input.start()

        self.vosk_path = Path.home()/"car1"/"src"/"my_car_models"/"vosk-model-en-us-0.22-lgraph"               #Vosk text-to-speech model
        self.listener = Model(str(self.vosk_path))
        self.cmd_recognizer = KaldiRecognizer(self.listener, 16000)    
        self.wake_recognizer =  KaldiRecognizer(self.listener, 16000, json.dumps(["listen patrick", "[unk]"])) #wake word: "listen patrick"
        self.wake_timer = self.create_timer(0.1, self.wake)

        #dict for obj ord in cmd
        self.ord_dict = {
            "zeroth":  0, "zero":  0, "0":   0,
            "first":   1, "one":   1, "1":   1,
            "second":  2, "two":   2, "2":   2,
            "third":   3, "three": 3, "3":   3,
            "fourth":  4, "four":  4, "4":   4,
            "fifth":   5, "five":  5, "5":   5,
            "sixth":   6, "six":   6, "6":   6,
            "seventh": 7, "seven": 7, "7":   7,
            "eighth":  8, "eight": 8, "8":   8,
            "ninth":   9, "nine":  9, "9":   9,
            "tenth":  10, "ten":  10, "10": 10
        }

    #process queued audio chunks (if any) to detect wake word.
    def wake(self):
        if self.status == "listen": #only listen to cmd when "listen"
            self.cmd_listen()
            return

        if self.status != "ready":  #only listen to wake word when "ready"
            return

        try:
            audio_chunk = self.audio_queue.get_nowait()
        except Empty:
            return

        if self.wake_recognizer.AcceptWaveform(audio_chunk):
            wake_word = json.loads(self.wake_recognizer.Result()).get("text", "")
            wake_word = wake_word.lower().strip()

            if wake_word == "listen patrick":
                self.audio_input.stop() #stop audio b4 mp3, to avoid chunks in audio queue.
                playsound(self.listen_sound_path, True)
                self.audio_queue = Queue()
                self.status = "listen"
                self.cmd_recognizer.Reset()
                self.audio_input.start()       
                
    #receive audio chunks from mic to audio_queue
    def audio_callback(self, indata, frames, time, status):
        if self.status == "ready" or self.status == "listen":
            self.audio_queue.put_nowait(bytes(indata))

    #listen to cmd and extract objects in cmd.
    #cmd must contain only 1 object.
    #valid cmd: action + order + object. No negation in cmd (dont, don't, or not). 
    def cmd_listen(self):
        if self.audio_queue.empty():
            return
        audio_chunk = self.audio_queue.get_nowait()

        #After a listening window, find out what objs are in cmd (pattern matching)
        if self.cmd_recognizer.AcceptWaveform(audio_chunk):
            cmd = json.loads(self.cmd_recognizer.Result()).get("text", "")
            cmd = cmd.lower().strip()

            if len(cmd) == 0: #if cmd is empty, return to wake word listen
                self.audio_input.stop()
                self.audio_queue = Queue()    
                self.status = "ready"
                self.wake_recognizer.Reset()
                self.audio_input.start()
                return

            act = r"\b(?:move|proceed|head|go|jump|navigate|find|search|seek)\b"
            negate = r"\b(?:don['’]?t|not)\b"
            if len(self.det_obj) > 0 and re.search(act, cmd) and not re.search(negate, cmd):
                #build up patterns
                cls = {cls_name.rsplit("_", 1)[0] for cls_name in self.det_obj}
                objs = "|".join(re.escape(name) for name in cls)
                ords = "|".join(map(re.escape, self.ord_dict))
                pats = [
                    rf"\b(?P<ord>{ords})\s+(?P<obj>{objs})\b", #e.g., first ball
                    rf"\b(?P<obj>{objs})\s+(?P<ord>{ords})\b"  #e.g., ball one/1
                ]

                #find 1 match between pattern and cmd. 
                mats = []
                for pat in pats:  
                    for mat in re.finditer(pat, cmd): 
                        mats.append(mat)
                if len(mats) == 1: #there's 1 match between pattern and cmd. 
                    mat = mats[0]
                    ord = self.ord_dict[mat["ord"]]
                    obj = f"{mat["obj"]}_{ord}"

                    if self.det_obj.get(obj) == None: #no such object
                        self.audio_input.stop()
                        playsound(self.fail_sound_path, True)
                        self.audio_queue = Queue()
                        self.status = "ready"
                        self.wake_recognizer.Reset()
                        self.audio_input.start()
                        return

                    self.audio_input.stop()
                    playsound(self.go_sound_path, True)
                    self.audio_queue = Queue()             

                    self.goal_pose = self.det_obj.get(obj)
                    self.follow_goal(self.goal_pose)
                else:   #no match between pattern and cmd. 
                    self.audio_input.stop()
                    playsound(self.fail_sound_path, True)
                    self.audio_queue = Queue()    
                    self.status = "ready"
                    self.wake_recognizer.Reset()
                    self.audio_input.start()
            else:   #if det_obj empty, no act, or a negation
                self.audio_input.stop()
                playsound(self.fail_sound_path, True)
                self.audio_queue = Queue()    
                self.status = "ready"
                self.wake_recognizer.Reset()
                self.audio_input.start()

    #check status of the mapping
    def nav_status_sub(self, msg:NavComplete):
        if msg.nav_complete and self.status == "wait":

            #if log exists, load it and populate det_obj
            if self.log_path.exists(): 
                with open(self.log_path, "r", encoding="utf-8") as file:
                    log = yaml.safe_load(file) or {}

                for obj_cls_name, coords in log.items():
                    obj_pose = self.create_pose_stamped("map", coords["x"], coords["y"])
                    self.det_obj[obj_cls_name] = obj_pose
                playsound(self.ready_sound_path, True)
                self.status = "ready"

    def follow_goal(self, goal: PoseStamped):
        self.status = "navigating"
        action_request = NavigateToPose.Goal()
        action_request.pose = goal
        action_request.pose.header.stamp = self.get_clock().now().to_msg() #prevent stale request

        goal_future = self.navigate_client.send_goal_async(action_request)
        goal_future.add_done_callback(self.follow_goal_response) 

    def follow_goal_response(self, future: Future):
        goal_response = future.result()
        if not goal_response.accepted or self.goal_pose == None: 
            #if goal rejected or no goal, retry until accepted 
            self.follow_goal(self.goal_pose)

    def follow_check(self, msg: Log):
        if self.status != "navigating" or self.goal_pose == None:
            return
        
        if msg.name == "controller_server" and (msg.msg == "Reached the goal!" or msg.msg == "Failed to make progress"):
            #if reach object
            playsound(self.found_sound_path, True)
            self.status = "ready"
            self.goal_pose = None
            self.wake_recognizer.Reset()
            self.audio_input.start()
        elif msg.name == "bt_navigator" and msg.msg in ["Goal failed", "Goal aborted", "Goal canceled"]: 
            #if fail to reach object, retry until do
            self.follow_goal(self.goal_pose)

    def create_pose_stamped(self, frame_id: str, p_x: float, p_y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = p_x
        pose.pose.position.y = p_y

        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(0, 0, 0) 
        return pose

def main(args=None):
    rclpy.init(args=args)
    follower = follower_node("follower_node")

    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == "__main__":
    main()