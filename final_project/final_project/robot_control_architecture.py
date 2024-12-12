import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pi, cos, sin
from transforms3d.euler import quat2euler
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from gtts import gTTS
import threading
import time
import os
import pygame
import tempfile
from enum import Enum, auto
import socket
import sys
import signal
from ament_index_python.packages import get_package_share_directory
import os

os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = ''
os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'
os.environ['FASTRTPS_USE_SHARED_MEMORY'] = '0'
package_share_dir = get_package_share_directory('final_project')
app = Flask(__name__,
           template_folder=os.path.join(package_share_dir, 'templates'),
           static_folder=os.path.join(package_share_dir, 'static'))

class RobotState(Enum):
    IDLE = auto()
    ROTATING = auto()
    MOVING = auto()
    CHECKING_SURROUNDINGS = auto()
    WAITING_FOR_NEXT_WAYPOINT = auto()

class AudioPlayer:
    def __init__(self):
        pygame.mixer.init()
        self.temp_dir = tempfile.mkdtemp()
        self.transition_channel = pygame.mixer.Channel(1)
        self.pre_sound_text = {
            4: "But wait, what's that growling sound?"
        }
        self.transition_sounds = {
            0: "",
            1: "",
            2: os.path.join(package_share_dir, 'audio', 'q2.mp3'),
            3: os.path.join(package_share_dir, 'audio', 'q3.mp3'),
            4: os.path.join(package_share_dir, 'audio', 'q1.mp3'),
            5: os.path.join(package_share_dir, 'audio', 'trimmed_q4.mp3')
        }

    def play_existing_audio(self, file_path, wait=True):
        try:
            sound = pygame.mixer.Sound(file_path)
            self.transition_channel.play(sound)
            if wait:
                while self.transition_channel.get_busy():
                    pygame.time.Clock().tick(10)
        except Exception as e:
            print(f"Error playing audio file: {e}")

    def create_and_play_audio(self, text):
        try:
            temp_file = os.path.join(self.temp_dir, f'question_{hash(text)}.mp3')
            if not os.path.exists(temp_file):
                tts = gTTS(text=text, lang='en')
                tts.save(temp_file)
            pygame.mixer.music.unload()
            pygame.mixer.music.load(temp_file)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        except Exception as e:
            print(f"Error playing audio: {e}")

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('quiz_turtlebot_controller')
        
        self.setup_lidar_subscriber()
        self.setup_motion_publisher()
        self.setup_additional_publishers()
        self.setup_pose_subscriber()
        self.initialize_variables()

    def setup_lidar_subscriber(self):
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            lidar_qos
        )
        self.latest_scan = None

    def setup_motion_publisher(self):
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)

    def setup_additional_publishers(self):
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def setup_pose_subscriber(self):
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            odom_qos
        )

    def initialize_variables(self):
        self.current_state = RobotState.IDLE
        self.current_pose = None
        self.current_yaw = None
        self.distance_threshold = 0.05
        self.angle_threshold = 0.5
        self.movement_complete = threading.Event()
        self.current_waypoint_index = 0
        self.waypoints = []

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        orientation = self.current_pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_yaw = quat2euler(quaternion, 'sxyz')
        self.generate_relative_waypoints()

    def generate_relative_waypoints(self):
        if not self.current_pose:
            return

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        orientation = self.current_pose.orientation
        _, _, current_yaw = quat2euler([orientation.w, orientation.x, orientation.y, orientation.z], 'sxyz')
        
        self.waypoints = [
            {'x': current_x + 0.25 * cos(current_yaw), 'y': current_y + 0.25 * sin(current_yaw)},
            {'x': current_x + 0.30 * cos(current_yaw), 'y': current_y + 0.30 * sin(current_yaw)},
            {'x': current_x + 0.40 * cos(current_yaw), 'y': current_y + 0.40 * sin(current_yaw)},
            {'x': current_x + 0.401 * cos(current_yaw), 'y': current_y + 0.401 * sin(current_yaw)},
            {'x': current_x + 0.402 * cos(current_yaw), 'y': current_y + 0.402 * sin(current_yaw)},
            {'x': current_x + 0.43 * cos(current_yaw), 'y': current_y + 0.43 * sin(current_yaw)}
        ]

    def update_state_machine(self):
        if not self.current_pose:
            return

        if self.current_state == RobotState.IDLE:
            self.handle_idle_state()
        elif self.current_state == RobotState.ROTATING:
            self.handle_rotation_state()
        elif self.current_state == RobotState.MOVING:
            self.handle_moving_state()
        elif self.current_state == RobotState.CHECKING_SURROUNDINGS:
            self.handle_checking_surroundings_state()
        elif self.current_state == RobotState.WAITING_FOR_NEXT_WAYPOINT:
            self.handle_waiting_state()

    def handle_idle_state(self):
        if self.current_waypoint_index < len(self.waypoints):
            self.transition_to_rotating()

    def handle_rotation_state(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        angle_to_goal = self.get_angle_to_goal(waypoint['x'], waypoint['y'])
        
        if abs(angle_to_goal) <= self.angle_threshold:
            self.transition_to_moving()
        else:
            self.rotate_towards_goal(angle_to_goal)

    def handle_moving_state(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        distance = self.get_distance_to_goal(waypoint['x'], waypoint['y'])
        
        if distance <= self.distance_threshold:
            if self.current_waypoint_index == 4:
                self.transition_to_checking_surroundings()
            else:
                self.transition_to_waiting()
        else:
            self.move_towards_goal(distance)

    def handle_checking_surroundings_state(self):
        self.do_360_turn()
        self.transition_to_waiting()

    def handle_waiting_state(self):
        self.movement_complete.set()
        self.current_waypoint_index += 1
        self.current_state = RobotState.IDLE

    def transition_to_rotating(self):
        self.current_state = RobotState.ROTATING

    def transition_to_moving(self):
        self.current_state = RobotState.MOVING

    def transition_to_checking_surroundings(self):
        self.current_state = RobotState.CHECKING_SURROUNDINGS

    def transition_to_waiting(self):
        self.current_state = RobotState.WAITING_FOR_NEXT_WAYPOINT

    def get_distance_to_goal(self, goal_x, goal_y):
        if not self.current_pose:
            return float('inf')
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        x_distance = abs(goal_x - current_x)
        y_distance = abs(goal_y - current_y)
        return max(x_distance, y_distance)

    def get_angle_to_goal(self, goal_x, goal_y):
        if not self.current_pose:
            return 0.0
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        desired_yaw = atan2(goal_y - current_y, goal_x - current_x)
        angle_diff = desired_yaw - self.current_yaw
        
        while angle_diff > pi:
            angle_diff -= 2 * pi
        while angle_diff < -pi:
            angle_diff += 2 * pi
        return angle_diff

    def do_360_turn(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.5
        total_rotation = 0.0
        rate = self.create_rate(10)

        while total_rotation < 2 * pi:
            self.publisher.publish(cmd_vel)
            total_rotation += abs(cmd_vel.angular.z) * 0.1
            rate.sleep()

        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)

    def rotate_towards_goal(self, angle_to_goal):
        cmd_vel = Twist()
        angular_speed = min(0.3, max(0.1, abs(angle_to_goal)))
        cmd_vel.angular.z = angular_speed if angle_to_goal > 0 else -angular_speed
        self.publisher.publish(cmd_vel)

    def move_towards_goal(self, distance):
        cmd_vel = Twist()
        linear_speed = min(0.15, max(0.05, distance))
        cmd_vel.linear.x = linear_speed
        
        angle_to_goal = self.get_angle_to_goal(
            self.waypoints[self.current_waypoint_index]['x'],
            self.waypoints[self.current_waypoint_index]['y']
        )
        if abs(angle_to_goal) > 0.05:
            cmd_vel.angular.z = 0.1 if angle_to_goal > 0 else -0.1
            
        self.publisher.publish(cmd_vel)

    def move_to_waypoint(self, waypoint_index):
        self.movement_complete.clear()
        
        if not self.current_pose or waypoint_index >= len(self.waypoints):
            self.movement_complete.set()
            return False
            
        waypoint = self.waypoints[waypoint_index]
        goal_x = waypoint['x']
        goal_y = waypoint['y']
        
        max_attempts = 300
        attempts = 0
        
        while self.get_distance_to_goal(goal_x, goal_y) > self.distance_threshold:
            if not rclpy.ok() or attempts >= max_attempts:
                stop_cmd = Twist()
                self.publisher.publish(stop_cmd)
                self.movement_complete.set()
                return False
            
            cmd_vel = Twist()
            angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
            distance = self.get_distance_to_goal(goal_x, goal_y)
            
            if abs(angle_to_goal) > self.angle_threshold:
                angular_speed = min(0.3, max(0.1, abs(angle_to_goal)))
                cmd_vel.angular.z = angular_speed if angle_to_goal > 0 else -angular_speed
                cmd_vel.linear.x = 0.0
            else:
                cmd_vel.angular.z = 0.0
                linear_speed = min(0.15, max(0.05, distance))
                cmd_vel.linear.x = linear_speed
            
                if abs(angle_to_goal) > 0.05:
                    cmd_vel.angular.z = 0.1 if angle_to_goal > 0 else -0.1

            self.publisher.publish(cmd_vel)
            time.sleep(0.1)
            attempts += 1
            
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        
        if waypoint_index == 4:
            self.do_360_turn()

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        
        time.sleep(0.5)
        self.movement_complete.set()
        return True

class Game:
    def __init__(self):
        self.players = {}
        self.answers = {}
        self.current_question = 0
        self.timer = None
        self.status = 'registration'
        self.current_waypoint = 0
        self.movement_thread = None
        self.audio_player = AudioPlayer()
        
        self.intro_text = "Hi, future surgeon! I'm Dr. Bolt, your robo-doc. Today, we're helping Alex, a soccer-loving snack fan who's in a bit of a medical pickle. Let's dive in and save the day, one organ at a time!"
        self.closing_text = "Great work, team! Alex is back in action and ready to conquer the world thanks to you, superstar surgeon! Until next time, stay sharp and keep saving lives!"
        
        self.questions = [
            {
                "q": "Uh-oh! Alex has been complaining about a headache and blurry vision after getting hit on the head by a soccer ball during practice. Which organ could be behind this mess?",
                "a": "brain",
                "type": "text"
            },
            {
                "q": "Alert! Alex can't see clearly and is rubbing their face like crazy! Things are getting blurry and itchy! Quick, what's the first thing we need to check?",
                "a": "eyes",
                "type": "text"
            },
            {
                "q": "Whoa! Alex's breathing sounds like a balloon letting out airsomething's not working right. What do we need to fix?",
                "a": "upper-right",
                "type": "dragdrop",
                "background": "https://static.vecteezy.com/system/resources/previews/035/376/549/non_2x/cute-boy-human-body-cartoon-vector.jpg",
                "tools": [
                    {
                        "id": "lung-diagram",
                        "img": "https://calendar.otc.edu/media/uploads/2024/07/lung-4.png"
                    },
                    {
                        "id": "oxygen-mask",
                        "img": "https://cdn-icons-png.flaticon.com/512/3926/3926002.png"
                    },
                    {
                        "id": "heart-monitor",
                        "img": "https://cdn-icons-png.flaticon.com/512/2464/2464543.png"
                    }
                ]
            },
            {
                "q": "Alex's heartbeat is slowing down let's work some magic and pump it back to normal before the alarms go wild!",
                "a": "5",
                "type": "heart-pump",
                "minPumps": 5
            },
            {
                "q": "Yikes! Alex's lunch didn't sit well, and now their stomach is in trouble. I need you to help me out grab your tool and draw a cut where we can safely remove the problem. Let's do this carefully!",
                "a": "drawing",
                "type": "draw"
            },
            {
                "q": "Alex is having trouble moving something after a big fall it's swollen and stiff. What part of their body should we check?",
                "a": "knee",
                "type": "text"
            }
        ]
        self.required_players = 1

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
socketio = SocketIO(app, cors_allowed_origins="*")
game = Game()

@app.route('/')
def index():
    return render_template('index.html')

def check_port(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(('0.0.0.0', port))
        result = True
    except:
        result = False
    sock.close()
    return result

def start_timer():
    for i in range(45, -1, -1):
        if game.timer != threading.current_thread():
            return
        socketio.emit('timer', i)
        if i == 0:
            check_answers()
        time.sleep(1)

def move_robot_and_continue():
    if game.current_question == 2:
        pre_text = game.audio_player.pre_sound_text.get(game.current_question)
        if pre_text:
            game.audio_player.create_and_play_audio(pre_text)
    
    transition_sound_path = game.audio_player.transition_sounds.get(game.current_question)
    if transition_sound_path:
        sound = pygame.mixer.Sound(transition_sound_path)
        game.audio_player.transition_channel.play(sound, loops=-1)
    
    success = turtlebot.move_to_waypoint(game.current_waypoint)
    if success:
        game.current_waypoint = (game.current_waypoint + 1) % len(turtlebot.waypoints)
        socketio.emit('robot_arrived')
        game.audio_player.transition_channel.stop()
        
        if game.current_question < len(game.questions):
            question_text = game.questions[game.current_question]["q"]
            question_data = {
                'question': question_text,
                'type': game.questions[game.current_question]["type"]
            }
            if game.questions[game.current_question]["type"] == "dragdrop":
                question_data['tools'] = game.questions[game.current_question]["tools"]
            if game.questions[game.current_question]["type"] == "heart-pump":
                question_data['minPumps'] = game.questions[game.current_question]["minPumps"]
            socketio.emit('question', question_data)
            game.audio_player.create_and_play_audio(question_text)
            game.timer = threading.Thread(target=start_timer)
            game.timer.start()
        else:
            show_results()

def check_answers():
    if game.current_question >= len(game.questions):
        show_results()
        return

    current_q = game.current_question
    if current_q not in game.answers:
        game.answers[current_q] = {}
    
    for player in game.players:
        if player not in game.answers[current_q]:
            if game.questions[current_q]["type"] in ["draw", "dragdrop", "heart-pump"]:
                game.answers[current_q][player] = ""
            else:
                game.answers[current_q][player] = ''

    game.current_question += 1

    if game.current_question < len(game.questions):
        game.movement_thread = threading.Thread(target=move_robot_and_continue)
        game.movement_thread.start()
    else:
        show_results()

def show_results():
    scores = {player: 0 for player in game.players}
    for q_num, answers in game.answers.items():
        if game.questions[q_num]["type"] == "draw":
            for player, answer in answers.items():
                if answer and answer != "":
                    scores[player] += 1
        elif game.questions[q_num]["type"] == "dragdrop":
            pass
        elif game.questions[q_num]["type"] == "heart-pump":
            min_pumps = game.questions[q_num]['minPumps']
            for player, answer in answers.items():
                try:
                    pumps = int(answer)
                    if pumps >= min_pumps:
                        scores[player] += 1
                except:
                    continue
        else:
            correct = game.questions[q_num]['a'].lower()
            for player, answer in answers.items():
                if answer.lower() == correct:
                    scores[player] += 1
    
    max_score = max(scores.values())
    winners = [player for player, score in scores.items() if score == max_score]
    
    threading.Thread(
        target=game.audio_player.create_and_play_audio,
        args=(game.closing_text,)
    ).start()
    
    socketio.emit('results', {
        'scores': scores,
        'winners': winners,
        'tied': len(winners) > 1
    })
    game.status = 'results'

@socketio.on('join')
def handle_join(name):
    if game.status == 'registration' and name not in game.players:
        game.players[name] = request.sid
        emit('update_players', {
            'players': list(game.players.keys()),
            'required': game.required_players
        }, broadcast=True)
        emit('join_success', {'name': name})

@socketio.on('start_game')
def handle_start():
    if len(game.players) >= game.required_players:
        game.status = 'quiz'
        game.current_question = 0
        game.answers = {}
        
        game.audio_player.create_and_play_audio(game.intro_text)
        
        game.movement_thread = threading.Thread(target=move_robot_and_continue)
        game.movement_thread.start()

@socketio.on('answer')
def handle_answer(data):
    if game.status != 'quiz':
        return
    
    q_num = game.current_question
    if q_num not in game.answers:
        game.answers[q_num] = {}
    
    player = data.get('player')
    if player in game.players and player not in game.answers[q_num]:
        game.answers[q_num][player] = data['answer']
        emit('answer_confirmed', {'player': player}, broadcast=True)
        
        if len(game.answers[q_num]) == len(game.players):
            game.timer = None
            check_answers()

def cleanup(signum, frame):
    print("Cleaning up...")
    try:
        if 'turtlebot' in globals():
            turtlebot.destroy_node()
        if 'executor' in globals():
            executor.shutdown()
        if 'ros_thread' in globals() and ros_thread.is_alive():
            ros_thread._stop()
    except Exception as e:
        print(f"Cleanup error: {e}")
    finally:
        rclpy.shutdown()
        sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

def main():
    rclpy.init()
    global turtlebot
    turtlebot = TurtleBotController()
    executor = SingleThreadedExecutor()
    executor.add_node(turtlebot)
    
    ros_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    ros_thread.start()
    
    try:
        port = 5000
        while not check_port(port) and port < 5010:
            port += 1
        
        if port >= 5010:
            print("No available ports found")
            sys.exit(1)
            
        print(f"Starting server on port {port}")
        socketio.run(app, host='0.0.0.0', port=port, debug=True)
    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
