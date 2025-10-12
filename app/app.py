from flask import Flask, request, jsonify, render_template
import requests
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# ROS 2 퍼블리셔 클래스
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        self.publisher_ = self.create_publisher(Path, '/robot_path', 10)
        self.get_logger().info('Path Publisher has been created.')

app = Flask(__name__)

# Kakao REST API 키 (반드시 REST API Key)
KAKAO_REST_KEY = '2b039dd4f2eab27eb1e047ef5f56d5d9'

# 모든 HTML 파일을 위한 라우트 추가
@app.route('/')
def home():
    return render_template('login.html')

@app.route('/login.html')
def login():
    return render_template('login.html')

@app.route('/home.html')
def home_page():
    return render_template('home.html')

@app.route('/navigation.html')
def navigation():
    return render_template('navigation.html')

@app.route('/done_navigation.html')
def done_navigation():
    return render_template('done_navigation.html')

@app.route('/report.html')
def report():
    return render_template('report.html')

@app.route('/profile.html')
def profile():
    return render_template('profile.html')
    
@app.route('/profile_dog.html')
def profile_dog():
    return render_template('profile_dog.html')
    
@app.route('/profile_human.html')
def profile_human():
    return render_template('profile_human.html')

# 기존 검색 기능 라우트
@app.route('/search')
def search():
    query = request.args.get('query', '')
    if not query:
        return jsonify({"documents": []})

    url = "https://dapi.kakao.com/v2/local/search/keyword.json"
    headers = {"Authorization": f"KakaoAK {KAKAO_REST_KEY}"}
    params = {"query": query, "size": 5}

    resp = requests.get(url, headers=headers, params=params)

    if resp.status_code == 200:
        data = resp.json()
        return jsonify(data)
    else:
        return jsonify({"documents": []}), resp.status_code

# ✅ 길찾기 기능 및 ROS 2 통신 라우트
@app.route('/route')
def get_route():
    start_x = "127.076886" 
    start_y = "37.631943"
    end_x = request.args.get('end_x')
    end_y = request.args.get('end_y')

    if not end_x or not end_y:
        return jsonify({"error": "목적지 정보가 누락되었습니다."}), 400
    
    url = "https://apis-navi.kakaomobility.com/v1/directions"
    headers = {
        "Authorization": f"KakaoAK {KAKAO_REST_KEY}",
        "Content-Type": "application/json"
    }
    params = {
        "origin": f"{start_x},{start_y}",
        "destination": f"{end_x},{end_y}"
    }

    resp = requests.get(url, headers=headers, params=params)

    if resp.status_code == 200:
        data = resp.json()
        
        # ROS 2로 경로 데이터 발행
        if not rclpy.ok():
            rclpy.init()
        
        node = PathPublisher()
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = node.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' # 로봇의 좌표계에 따라 수정 필요
        
        # 카카오 API에서 받은 vertexes를 ROS 2 Path 메시지로 변환
        if data.get('routes') and data['routes'][0].get('sections'):
            for section in data['routes'][0]['sections']:
                for road in section['roads']:
                    for i in range(0, len(road['vertexes']), 2):
                        lng = road['vertexes'][i]
                        lat = road['vertexes'][i+1]
                        
                        # ROS 2 PoseStamped 메시지 생성
                        pose = PoseStamped()
                        pose.header = path_msg.header
                        pose.pose.position.x = float(lng)
                        pose.pose.position.y = float(lat)
                        pose.pose.position.z = 0.0
                        path_msg.poses.append(pose)
        
        node.publisher_.publish(path_msg)
        node.destroy_node()
        rclpy.shutdown()
        
        return jsonify(data)
    else:
        return jsonify({"error": "길찾기 실패"}), resp.status_code

if __name__ == '__main__':
    rclpy.init(args=None)
    # ✅ use_reloader=False 옵션 추가
    app.run(debug=True, use_reloader=False)