 <img width="336" height="323" alt="Image" src="https://github.com/user-attachments/assets/4b94c81e-fb31-4ce5-a0a3-2b706d18fe3b" />

## **PUPILOT**

---

## **1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : 인공지능 자율주행 기반의 스마트 유모차 로봇 개발
- 프로젝트 정의 : 보호자 편의성과 반려견 복지를 향상시키기 위한 자율주행 스마트 유모차와 AI 기반 강아지 감정 분석 산책 앱 개발
</br>

**1-2. 개발 배경 및 필요성**
- 최근 반려동물 양육 가구수는 꾸준히 증가하고 있으며, 특히 1인 가구와 고령층을 중심으로 반려견을 가족으로 여기는 문화가 확산되고 있음. 따라서 노령견이나 거동이 어려운 양육자를 지닌 반려견을 위한 유모차 수요가 급증함. 그러나 기존 제품은 대부분 수동식으로, 보호자에게 신체적 부담과 제어의 한계가 큼. 이에 따라 AI 자율주행, 원격 제어 등 스마트 기술을 접목한 차세대 반려동물 전용 유모차의 필요함.

**1-3. 프로젝트 특장점**
- AI 기반 자율주행 기능 탑재
- 원격 제어 및 위치 추적 기능 제공
- 반려견 반응 기반 스마트 보조 기능 구현
- 스마트폰 앱과의 연동성

**1-4. 주요 기능**
- 자율주행 : gps, imu, 등을 활용해 자동 경로 추적 및 장애물 회피 주행
- 원격제어 : 스마트폰 앱을 통해 실시간 위치 파악, 출발 정지 방향 제어 가능
- 스마트 보조 : 반려견의 움직임 등을 실시간 전달 및 감지
- 비전처리 : 반려견의 움직임을 분석하여 실시간 유모차 제어
- 모터 제어 : PID 알고리즘 및 엔코더 기반 정밀 속도, 방향 제어

**1-5. 기대 효과 및 활용 분야**
- 기술적 측면 : 자율주행, 비전 인식, IoT 기반 제어 등 첨단 기술 융합을 통한 스마트 펫 모빌리티 실현, 반려동물의 상태 인식 기반 주행 제어 등 AI 활용의 새로운 응용 가능성 제시, SLAM, 딥러닝 객체 인식, PID 제어 등 최신 기술을 실제 생활 환경에 적용
- 비즈니스적 측면 : 급성장 중인 펫테크 시장에서 경쟁력 있는 스마트 유모차 제품으로 상용화 가능성 확보, 앱 기반 데이터 수집, 헬스케어 연동 등을 통한 구독형 서비스 모델 구축 가능
- 사회적 측면 : 고령자, 장애인 보호자 등 다양한 계층의 외출 편의성 증대 및 삶의 질 향상, 반려동물을 가족처럼 여기는 문화에 맞춘 정서적 교감 및 복지 증진 기여, 반려동물 중심의 안전하고 쾌적한 산책 문화 조성
- 활용 분야 : 학술·연구, 유모차 대여 서비스, 헬스케어, 관광 서비스 등
  
**1-6. 기술 스택**
- 프론트엔드 : html, css, json
- 백엔드 : flask
- AI/ML : PyTorch

---
## **2. 팀원 소개**


| **김수현** | **김기현** | **류성훈** |
|:---:|:---:|:---:|
| • 강아지 감정 분석 모델 개발 <br> • 전용 애플리케이션 개발 <br> • 3D SLAM | • 유모차 전자부품 연결부 설계 <br> • 유모차 외장 설계 <br> • 회로 제작 | • 다중 센서 융합 기반 경로추종 <br> • ROS2 기반 자율주행  <br> • 2D SLAM <br> |

---
## **3. 시스템 구성도**

- 서비스 구성도
<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/28fc8453-d1a0-4184-8fd0-130d93d18545" />


- 제어 흐름도
<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/76e3347b-6d94-491e-8aeb-a7b4601c54d5" />

---
## **4. 작품 소개영상**

[![2025년 한이음 드림업 공모전] [25_HC287] 인공지능 자율주행 기반의 스마트 유모차 로봇 개발] [<img width="2558" height="1427" alt="Image" src="https://github.com/user-attachments/assets/c548f117-3aa6-4840-814d-241dcf2a78cc" />](https://www.youtube.com/watch?v=Nd-ArHrq8iM)

---
## **5. 핵심 소스코드**

```python
app.py
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


cameara_server.py
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

camera_server
import cv2
import onnxruntime as ort
import numpy as np
from flask import Flask, Response
import time
from PIL import ImageFont, ImageDraw, Image

# Flask 앱 인스턴스 생성
app = Flask(__name__)

# ONNX 모델 로드
# ✅ dog_emotion_prior.onnx 모델 파일을 로드합니다.
# 모델 파일의 경로는 camera_server.py와 같은 위치에 있어야 합니다.
try:
    # ONNX Runtime 세션 생성 (CPU 실행 프로바이더만 사용)
    session = ort.InferenceSession("dog_emotion_prior.onnx", 
                                   providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    print("ONNX 모델이 성공적으로 로드되었습니다.")
except Exception as e:
    print(f"ONNX 모델 로드 중 오류 발생: {e}")
    session = None

# 웹캠 설정 및 해상도 조정
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 감정 레이블 (모델의 출력 순서에 맞게 수정)
EMOTION_LABELS = ['angry', 'happy', 'relax', 'sad']

# ✅ 한글 폰트 로드
# 시스템에 설치된 한글 폰트 파일 경로로 변경해야 합니다.
# 예: 나눔고딕 (보통 /usr/share/fonts/truetype/nanum/NanumGothic.ttf)
# 아래 경로는 예시이며, 실제 폰트 파일 경로를 확인해주세요.
font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
try:
    font = ImageFont.truetype(font_path, 20)
except IOError:
    print(f"폰트 파일 '{font_path}'를 찾을 수 없습니다. 기본 폰트로 대체합니다.")
    font = ImageFont.load_default()

def analyze_emotion(frame):
    """프레임에서 강아지 감정을 분석하고 결과를 반환하는 함수"""
    if session is None:
        return "모델 로드 실패"
        
    try:
        # 모델 입력 크기에 맞게 이미지 전처리 (예: 224x224)
        input_size = session.get_inputs()[0].shape[2:4]
        processed_frame = cv2.resize(frame, tuple(input_size))
        
        # 모델 입력 형식에 맞게 변환 (예: (1, 3, 224, 224), float32)
        # BGR -> RGB 변환
        processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
        processed_frame = processed_frame.transpose((2, 0, 1))
        processed_frame = np.expand_dims(processed_frame, axis=0).astype(np.float32)

        # 모델 추론
        outputs = session.run([output_name], {input_name: processed_frame})
        emotion_probs = outputs[0][0]
        
        # 가장 높은 확률의 감정 찾기
        predicted_index = np.argmax(emotion_probs)
        predicted_emotion = EMOTION_LABELS[predicted_index]
        
        return predicted_emotion
    except Exception as e:
        print(f"감정 분석 중 오류 발생: {e}")
        return "분석 오류"

@app.route('/emotion_with_video_feed')
def emotion_with_video_feed():
    """감정 분석 결과가 오버레이된 비디오 스트림을 위한 HTTP 응답 라우트"""
    def generate_frames():
        # 감정 분석은 모든 프레임에서 실행하면 CPU 사용량이 높아지므로, 1초에 한 번만 실행합니다.
        emotion = "분석 중..."
        last_analysis_time = time.time()
        
        while True:
            success, frame = camera.read()
            if not success:
                break
            
            current_time = time.time()
            # 1초마다 감정 분석을 수행하여 CPU 사용량을 낮춥니다.
            if current_time - last_analysis_time >= 1:
                emotion = analyze_emotion(frame)
                last_analysis_time = current_time

            # ✅ OpenCV 이미지(numpy.ndarray)를 PIL 이미지로 변환
            pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_image)
            
            # ✅ PIL을 사용하여 한글 텍스트를 그립니다.
            # 폰트 색상(RGB)과 위치를 지정합니다.
            draw.text((10, 30), f"{emotion}", font=font, fill=(0, 255, 0))

            # ✅ PIL 이미지를 다시 OpenCV 이미지(numpy.ndarray)로 변환
            frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 재시작 기능을 비활성화하여 충돌 방지
    app.run(host='0.0.0.0', port=5001, debug=True, use_reloader=False)
```

