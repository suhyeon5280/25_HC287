import json
import numpy as np
import pandas as pd
from math import isclose

def analyze_and_interpolate_route(file_path):
    """
    경로 데이터를 분석하고 선형 보간을 통해 촘촘한 경로를 생성한 후, 
    안내 정보를 매칭하여 DataFrame 형태로 반환합니다. (중복 문제 수정 완료!)
    """
    try:
        # 1. JSON 파일 로드
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        return "에러: 파일을 찾을 수 없어요! 파일 경로를 확인해 줘. 😭"
    except json.JSONDecodeError:
        return "에러: JSON 파일 형식이 이상해! 파일을 다시 확인해 줘. 😱"

    route = data['routes'][0]
    sections = route['sections'][0]
    
    # 원본 경로 포인트 (보간 대상)
    original_points = data['all_points']
    
    # 안내 지점 정보 (매칭 대상)
    guidance_list = sections['guides']

    # 2. 선형 보간 (Linear Interpolation)
    interpolated_points = []
    
    # 📌 수정된 로직 1: 첫 번째 포인트는 무조건 포함합니다.
    if not original_points:
        return pd.DataFrame(columns=['위도', '경도', '안내'])
        
    interpolated_points.append(original_points[0]) 

    # 각 원본 포인트 사이에 10개의 구간을 만들어 촘촘하게 만듭니다.
    num_segments = 10 
    
    # 보간 파라미터 t를 생성 (시작점 t=0을 제외하고, 끝점 t=1을 포함합니다.)
    # [0.1, 0.2, ..., 1.0]
    t = np.linspace(0, 1, num_segments + 1)[1:]

    for i in range(len(original_points) - 1):
        start_point = original_points[i]
        end_point = original_points[i+1]
        
        start_lat = start_point['lat']
        start_lng = start_point['lng']
        end_lat = end_point['lat']
        end_lng = end_point['lng']

        # 📌 수정된 로직 2: 시작점과 끝점이 같으면 보간하지 않고 넘어갑니다. (중복 제거 핵심!)
        if isclose(start_lat, end_lat, abs_tol=1e-9) and isclose(start_lng, end_lng, abs_tol=1e-9):
            continue
        
        # 선형 보간 (t=0, 즉 시작점은 이미 추가했으므로 t>0 지점만 계산)
        lat_interp = start_lat + t * (end_lat - start_lat)
        lng_interp = start_lng + t * (end_lng - start_lng)

        for lat, lng in zip(lat_interp, lng_interp):
            interpolated_points.append({'lat': lat, 'lng': lng})

    # 3. DataFrame 생성 및 안내(Guidance) 컬럼 초기화
    df_route = pd.DataFrame(interpolated_points)
    df_route['안내'] = '직진 (Straight)'
    
    # 좌표 비교를 위한 허용 오차 (부동소수점 비교)
    TOLERANCE = 1e-9

    # 4. 안내 지점 정보 매칭
    for guide in guidance_list:
        guide_lat = guide['y'] # JSON 파일의 가이던스는 'y'가 위도(lat)
        guide_lng = guide['x'] # JSON 파일의 가이던스는 'x'가 경도(lng)
        instruction = guide['guidance']
        
        # 보간된 경로에서 가이던스 좌표와 일치하는 포인트를 찾습니다.
        match_index = df_route[
            df_route.apply(
                lambda row: isclose(row['lat'], guide_lat, abs_tol=TOLERANCE) and \
                            isclose(row['lng'], guide_lng, abs_tol=TOLERANCE),
                axis=1
            )
        ].index

        # 매칭되는 인덱스에 안내 정보를 업데이트합니다.
        if not match_index.empty:
            df_route.loc[match_index, '안내'] = instruction

    # 5. 컬럼 이름 변경 및 순서 조정
    df_final = df_route.rename(columns={
        'lat': '위도',
        'lng': '경도'
    })[['위도', '경도', '안내']]
    
    return df_final

# =======================================================
# --- 실행 부분: 분석, 출력 및 파일 저장 ---
# =======================================================

file_path = "route_data.json"
result_df = analyze_and_interpolate_route(file_path)

if isinstance(result_df, pd.DataFrame):
    print(f"총 {len(result_df)}개의 경로 포인트가 포함된 표를 만들었어! ✨")
    
    # 터미널 출력
    print("\n--- 경로 포인트 (상위 5개) ---")
    print(result_df.head().to_markdown(index=False))
    print("\n--- 경로 포인트 (하위 5개) ---")
    print(result_df.tail().to_markdown(index=False))

    # 🎀 6. 파일 저장 🎀

    # CSV 파일로 저장: 엑셀에서 보기 편해!
    output_csv_file = "interpolated_route_data_final.csv"
    result_df.to_csv(output_csv_file, index=False, encoding='utf-8-sig') 

    # JSON 파일로 저장: 다른 프로그램에서 데이터로 쓰기 편해!
    output_json_file = "interpolated_route_data_final.json"
    # orient='records' 옵션으로 리스트 안에 딕셔너리 형태로 저장돼!
    result_df.to_json(output_json_file, orient='records', indent=4, force_ascii=False)

    print(f"\n파일 저장 완료! 🥳")
    print(f"**CSV 파일**은 '{output_csv_file}'에 저장되었고,")
    print(f"**JSON 파일**은 '{output_json_file}'에 저장되었단다! 완벽해! 💕")

else:
    # 에러 메시지 출력
    print(result_df)