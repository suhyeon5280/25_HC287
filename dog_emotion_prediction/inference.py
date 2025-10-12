import onnxruntime as ort
import numpy as np
from PIL import Image

# --- 설정 ---
ONNX_PATH = "/home/suhyeon/Downloads/dog_emotion_prior.onnx"  # 내보낸 ONNX 파일 경로
CLASSES = ["angry", "happy", "relaxed", "sad"]
IMG_SIZE = 224

# --- 세션 준비 ---
sess = ort.InferenceSession(ONNX_PATH, providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
in_name = sess.get_inputs()[0].name
out_name = sess.get_outputs()[0].name   # logits

# --- 전처리 함수 ---
def preprocess(path, size=IMG_SIZE):
    img = Image.open(path).convert("RGB")
    resize_side = int(size * 1.15)
    img = img.resize((resize_side, resize_side), Image.BILINEAR)
    # center crop
    left = (resize_side - size) // 2
    top  = (resize_side - size) // 2
    img = img.crop((left, top, left+size, top+size))

    arr = np.asarray(img).astype("float32")/255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    arr = (arr - mean) / std
    arr = arr.transpose(2,0,1)        # [3,H,W]
    return arr[np.newaxis,...]        # [1,3,224,224]

# --- 추론 ---
def predict_one(img_path):
    x = preprocess(img_path)
    logits = sess.run([out_name], {in_name: x})[0]   # [1,num_classes]
    probs = np.exp(logits - logits.max()) / np.exp(logits - logits.max()).sum()
    pred_idx = int(probs.argmax())
    return CLASSES[pred_idx], float(probs[0,pred_idx]), dict(zip(CLASSES, probs[0].round(4)))

# 사용 예시
img_path = "/home/suhyeon/smiling-dog.jpg"
label, conf, all_probs = predict_one(img_path)
print("Pred:", label, "Conf:", conf)
print("All probs:", all_probs)
