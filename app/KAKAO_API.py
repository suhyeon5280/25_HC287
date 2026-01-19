import json
import requests
from flask import Flask, jsonify

app = Flask(__name__)

KAKAO_API_KEY = "" #api keyy 넣기

@app.route("/")
def home():
    return "카카오 길찾기 API 서버 실행 중 🚀 /route 로 접속하면 경로 JSON을 확인할 수 있습니다."

@app.route("/route")
def get_route():
    start_x = 127.076886
    start_y = 37.631943
    end_x = 127.075990
    end_y = 37.631450

    url = "https://apis-navi.kakaomobility.com/v1/directions"
    headers = {"Authorization": f"KakaoAK {KAKAO_API_KEY}"}
    params = {
        "origin": f"{start_x},{start_y}",
        "destination": f"{end_x},{end_y}",
        "priority": "TIME"
    }

    try:
        res = requests.get(url, headers=headers, params=params)
        res.raise_for_status()
        data = res.json()
    except requests.exceptions.RequestException as e:
        return jsonify({"error": "Kakao API 호출 실패", "detail": str(e)}), 500

    all_points = []
    try:
        for section in data.get("routes", [])[0].get("sections", []):
            for road in section.get("roads", []):
                vertexes = road.get("vertexes", [])
                for i in range(0, len(vertexes), 2):
                    lng = vertexes[i]
                    lat = vertexes[i + 1]
                    all_points.append({"lat": lat, "lng": lng})
    except Exception as e:
        return jsonify({"error": "좌표 변환 실패", "detail": str(e)}), 500

    data["all_points"] = all_points

    # 🔹 JSON 파일로 저장
    try:
        with open("route_data.json", "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
    except Exception as e:
        return jsonify({"error": "파일 저장 실패", "detail": str(e)}), 500

    return jsonify(data)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)
