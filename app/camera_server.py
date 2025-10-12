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