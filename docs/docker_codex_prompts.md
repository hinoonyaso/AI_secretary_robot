# JetRover "Rover" — Docker 미구현 부분 Codex 프롬프트 모음

> **실행 순서**: Prompt A → B → C → D → E → F → G (순서대로 진행)
> **목표**: 현재 Dockerfile의 미구현 부분(VLM, OCR, 임베딩, DB, Compose) 완성
> **환경**: Jetson Orin Nano 8GB / JetPack 6.x / CUDA 12.2 / ARM64 / Ubuntu 22.04
> **기반**: 기존 `docker/Dockerfile` (base + ai-runtime + vision-runtime 완성됨)

---

## 📦 Prompt A: Dockerfile — VLM (Moondream) Stage 추가

```
[Context]
- Current Dockerfile has 3 stages: base, ai-runtime, vision-runtime
- vision-runtime includes YOLO TensorRT but lacks VLM (Vision-Language Model)
- Goal: Add moondream:1.8b for object description and open-ended VQA
- Output: Add new stage `FROM vision-runtime AS vlm-runtime` to docker/Dockerfile

[Critical Requirements]
⚠️ Moondream 1.8b는 PyTorch 모델이므로 ONNX로 변환해야 합니다.
그러나 Jetson에서 변환 시 메모리 부족으로 실패할 수 있으므로,
사전 변환된 ONNX 모델을 다운로드하는 방식을 권장합니다.

대안 1: llama.cpp vision 모델 사용 (권장)
- llama.cpp는 이미 ai-runtime에 설치되어 있음
- moondream2 GGUF 모델 사용 (llava 아키텍처 호환)
- https://huggingface.co/vikhyatk/moondream2/tree/main

대안 2: ONNX Runtime + moondream ONNX
- HuggingFace에서 moondream ONNX 모델 검색
- 없으면 x86 서버에서 변환 후 다운로드

[Task]
Write the FOURTH STAGE (`FROM vision-runtime AS vlm-runtime`) of the Dockerfile.

Requirements:

1. [Block A] VLM 모델 준비 (llama.cpp 기반 — 권장):
   a. 환경 변수 설정:
      VLM_MODEL_PATH=/opt/rover/models/vlm/moondream2-q4_k_m.gguf
      VLM_MMPROJ_PATH=/opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf

   b. 모델 다운로드 (런타임 마운트 대신 번들):
      mkdir -p /opt/rover/models/vlm
      wget -O /opt/rover/models/vlm/moondream2-q4_k_m.gguf \
        https://huggingface.co/vikhyatk/moondream2-GGUF/resolve/main/moondream2-text-model-q4_k_m.gguf
      wget -O /opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf \
        https://huggingface.co/vikhyatk/moondream2-GGUF/resolve/main/moondream2-mmproj-f16.gguf

   c. 검증:
      llama-cli --version  # 이미 설치되어 있음 확인
      test -f /opt/rover/models/vlm/moondream2-q4_k_m.gguf
      test -f /opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf

2. [Block B] VLM ROS2 노드 C++ 작성 (/opt/rover/src/vlm_ros_node.cpp):
   a. 클래스: VlmNode : public rclcpp::Node

   b. 구독:
      - /camera/color/image_raw (sensor_msgs/Image)
      - /vlm/query (std_msgs/String) — 질문 텍스트

   c. 발행:
      - /vlm/response (std_msgs/String) — JSON 응답
        형식: {"query": "...", "answer": "...", "timestamp": 123.456}

   d. 로직:
      - 이미지 수신 시 최신 프레임을 버퍼에 저장 (1개만 유지)
      - 질문 수신 시:
        1. 버퍼의 최신 이미지를 JPEG로 인코딩
        2. llama-server HTTP API 호출 (포트 8081)
           POST /completion
           {
             "prompt": "USER: <image>\n{query}\nASSISTANT:",
             "image_data": [{"data": "base64_jpeg", "id": 0}],
             "temperature": 0.1,
             "n_predict": 128
           }
        3. 응답 파싱 후 /vlm/response로 발행

   e. llama-server 시작 로직:
      - 노드 시작 시 별도 스레드에서 llama-server 실행:
        llama-server \
          --model /opt/rover/models/vlm/moondream2-q4_k_m.gguf \
          --mmproj /opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf \
          --port 8081 \
          --ctx-size 2048 \
          --n-gpu-layers 99
      - 노드 종료 시 llama-server 프로세스도 종료

3. [Block C] CMakeLists.txt for vlm_ros_node:
   - find_package(ament_cmake REQUIRED)
   - find_package(rclcpp REQUIRED)
   - find_package(sensor_msgs REQUIRED)
   - find_package(std_msgs REQUIRED)
   - find_package(OpenCV REQUIRED)
   - find_package(CURL REQUIRED) — HTTP 통신용
   - executable: vlm_ros_node
   - link: ${OpenCV_LIBS}, ${CURL_LIBRARIES}

4. [Block D] 빌드 검증:
   cd /opt/rover/vlm_ws
   cmake -B build -DCMAKE_PREFIX_PATH=/opt/ros/humble
   cmake --build build --target vlm_ros_node -j2
   install -m 0755 build/vlm_ros_node /usr/local/bin/vlm_ros_node

5. Cleanup:
   rm -rf /opt/rover/vlm_ws /var/lib/apt/lists/*

[Alternative — ONNX Runtime 기반]
만약 llama.cpp vision 지원이 불안정하면:
1. pip3 install transformers einops timm
2. Python wrapper로 moondream2 모델 로드
3. ROS2 Python 노드로 구현 (rclpy 사용)

[Constraints]
- VLM 모델은 번들 (다운로드 포함), 마운트 아님
- llama-server는 백그라운드 프로세스로 실행 (systemd 아님)
- 메모리 사용량: VRAM < 1.5GB, RAM < 600MB (q4_k_m 양자화)

[Verification]
RUN bash -c "test -f /opt/rover/models/vlm/moondream2-q4_k_m.gguf && \
             test -f /usr/local/bin/vlm_ros_node && \
             echo 'VLM stage OK'"

[Output]
- Dockerfile에 `FROM vision-runtime AS vlm-runtime` 스테이지 추가
- 총 라인 수: ~200줄 추가 예상
```

---

## 📦 Prompt B: Dockerfile — OCR (PaddleOCR) Stage 추가

```
[Context]
- Previous stage: vlm-runtime
- Goal: Add PaddleOCR PP-OCRv3 for text detection and recognition (Korean support)
- Output: Add stage `FROM vlm-runtime AS ocr-runtime` to docker/Dockerfile

[Task]
Write the FIFTH STAGE (`FROM vlm-runtime AS ocr-runtime`).

Requirements:

1. [Block A] PaddleOCR 설치:
   a. Python 패키지 설치:
      pip3 install --no-cache-dir \
        paddlepaddle==2.6.0 \
        paddleocr==2.7.0.3 \
        shapely pyclipper lmdb tqdm

   b. 모델 자동 다운로드 (첫 실행 시):
      python3 -c "from paddleocr import PaddleOCR; \
                  ocr = PaddleOCR(use_angle_cls=True, lang='korean'); \
                  print('Model downloaded to ~/.paddleocr')"

   c. 모델을 영구 경로로 복사:
      mkdir -p /opt/rover/models/ocr
      cp -r ~/.paddleocr/whl/det /opt/rover/models/ocr/
      cp -r ~/.paddleocr/whl/rec/korean /opt/rover/models/ocr/rec_korean
      cp -r ~/.paddleocr/whl/cls /opt/rover/models/ocr/

   d. Cleanup:
      rm -rf ~/.paddleocr

2. [Block B] OCR ROS2 노드 Python 작성 (/opt/rover/scripts/ocr_node.py):
   a. #!/usr/bin/env python3 shebang

   b. 클래스: OcrNode(Node)

   c. 구독:
      - /camera/color/image_raw (sensor_msgs/Image) — 이미지 입력
      - /ocr/trigger (std_msgs/Empty) — OCR 실행 트리거

   d. 발행:
      - /ocr/result (std_msgs/String) — JSON 출력
        형식: {
          "text_blocks": [
            {"text": "인식된 텍스트", "confidence": 0.95, "bbox": [[x1,y1], [x2,y2], ...]},
            ...
          ],
          "timestamp": 123.456
        }

   e. OCR 엔진 초기화:
      from paddleocr import PaddleOCR
      self.ocr = PaddleOCR(
          det_model_dir='/opt/rover/models/ocr/det',
          rec_model_dir='/opt/rover/models/ocr/rec_korean',
          cls_model_dir='/opt/rover/models/ocr/cls',
          use_angle_cls=True,
          lang='korean',
          use_gpu=True,
          show_log=False
      )

   f. 이미지 처리:
      - cv_bridge로 ROS Image → OpenCV Mat
      - PaddleOCR 실행: result = self.ocr.ocr(img, cls=True)
      - JSON 직렬화 후 발행

   g. 실행 권한:
      chmod +x /opt/rover/scripts/ocr_node.py

3. [Block C] Launch 파일 작성 (/opt/rover/launch/ocr.launch.py):
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ocr_node',  # 패키지명은 임시
               executable='ocr_node.py',
               name='ocr_node',
               output='screen',
               parameters=[
                   {'det_model_dir': '/opt/rover/models/ocr/det'},
                   {'rec_model_dir': '/opt/rover/models/ocr/rec_korean'},
                   {'cls_model_dir': '/opt/rover/models/ocr/cls'},
               ]
           )
       ])

4. [Block D] 검증:
   python3 /opt/rover/scripts/ocr_node.py --help || echo "OCR node OK"
   test -d /opt/rover/models/ocr/det
   test -d /opt/rover/models/ocr/rec_korean

5. Cleanup:
   pip3 cache purge
   rm -rf /var/lib/apt/lists/*

[Constraints]
- PaddlePaddle는 CPU 버전 사용 (GPU 버전은 11GB+ 이미지 크기 증가)
- 한국어 모델만 다운로드 (영어, 중국어 제외)
- 모델 파일 총 크기: ~80MB

[Alternative — TensorRT 최적화]
PaddleOCR 모델을 TensorRT로 변환 시:
- paddle2onnx 사용
- trtexec로 FP16 엔진 생성
- 추론 속도 2~3배 향상 (20ms → 8ms)
⚠️ 복잡도가 높으므로 Phase 2에서 진행 권장

[Verification]
RUN python3 -c "from paddleocr import PaddleOCR; print('PaddleOCR OK')"

[Output]
- Dockerfile에 `FROM vlm-runtime AS ocr-runtime` 스테이지 추가
- 총 라인 수: ~100줄 추가 예상
```

---

## 📦 Prompt C: Dockerfile — 임베딩 & Wake Word Stage

```
[Context]
- Previous stage: ocr-runtime
- Goal: Add KoSimCSE-RoBERTa embedding model and Porcupine wake word engine
- Output: Add final stage `FROM ocr-runtime AS full-runtime` to docker/Dockerfile

[Task]
Write the SIXTH and FINAL STAGE (`FROM ocr-runtime AS full-runtime`).

Requirements:

1. [Block A] KoSimCSE 임베딩 모델 ONNX 변환 & 설치:

   ⚠️ ONNX 변환은 Jetson에서 직접 수행하면 메모리 초과 가능.
   사전 변환된 모델이 없으면 x86 서버에서 변환 후 다운로드.

   a. Transformers 설치 (변환용):
      pip3 install --no-cache-dir transformers torch --index-url https://download.pytorch.org/whl/cpu

   b. Python 변환 스크립트 실행 (/tmp/convert_kosimcse.py):
      ```python
      from transformers import AutoTokenizer, AutoModel
      import torch
      import onnx

      model_name = "BM-K/KoSimCSE-roberta"
      tokenizer = AutoTokenizer.from_pretrained(model_name)
      model = AutoModel.from_pretrained(model_name)

      dummy_input = tokenizer("테스트", return_tensors="pt")

      torch.onnx.export(
          model,
          (dummy_input['input_ids'], dummy_input['attention_mask']),
          "/opt/rover/models/embedding/kosimcse-roberta.onnx",
          input_names=['input_ids', 'attention_mask'],
          output_names=['last_hidden_state'],
          dynamic_axes={
              'input_ids': {0: 'batch', 1: 'sequence'},
              'attention_mask': {0: 'batch', 1: 'sequence'},
              'last_hidden_state': {0: 'batch', 1: 'sequence'}
          },
          opset_version=14
      )
      print("ONNX export complete")
      ```

   c. 검증:
      python3 -c "import onnx; m = onnx.load('/opt/rover/models/embedding/kosimcse-roberta.onnx'); \
                  print('ONNX model OK, inputs:', [i.name for i in m.graph.input])"

   d. Cleanup:
      pip3 uninstall -y torch torchvision  # PyTorch 제거 (변환만 필요)
      rm /tmp/convert_kosimcse.py

2. [Block B] Porcupine Wake Word 엔진 설치:

   a. Porcupine SDK 다운로드:
      mkdir -p /opt/porcupine /opt/rover/models/wake_word
      wget -O /tmp/porcupine.tar.gz \
        https://github.com/Picovoice/porcupine/archive/refs/tags/v3.0.2.tar.gz
      tar -xzf /tmp/porcupine.tar.gz -C /tmp

   b. ARM64 라이브러리 복사:
      cp /tmp/porcupine-3.0.2/lib/linux/arm64/libpv_porcupine.so /usr/local/lib/
      cp -r /tmp/porcupine-3.0.2/lib/common /opt/porcupine/
      cp -r /tmp/porcupine-3.0.2/resources /opt/porcupine/
      ldconfig

   c. 한국어 wake word 모델 복사:
      # "안녕 로버" 커스텀 모델이 있다면:
      # cp custom_wake_word_ko.ppn /opt/rover/models/wake_word/
      # 기본 영어 모델 사용:
      cp /tmp/porcupine-3.0.2/resources/keyword_files/linux/porcupine_linux.ppn \
         /opt/rover/models/wake_word/porcupine.ppn

   d. Python 바인딩 설치:
      pip3 install --no-cache-dir pvporcupine==3.0.2

   e. 검증:
      python3 -c "import pvporcupine; \
                  p = pvporcupine.create(keywords=['porcupine']); \
                  print('Porcupine OK, version:', p.version)"

   f. Cleanup:
      rm -rf /tmp/porcupine*

3. [Block C] 임베딩 서비스 C++ 노드 (llama-embedding 사용):

   ⚠️ KoSimCSE ONNX는 복잡하므로, llama-embedding 바이너리를 활용.
   llama.cpp는 RoBERTa GGUF 모델 지원 (v1.0+).

   a. KoSimCSE GGUF 변환 (사전 변환 필요):
      # Host에서 llama.cpp convert-hf-to-gguf.py 실행
      # 또는 HuggingFace에서 GGUF 검색
      wget -O /opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf \
        https://huggingface.co/{user}/KoSimCSE-GGUF/resolve/main/kosimcse-roberta-q8_0.gguf

      ⚠️ 위 URL은 예시입니다. 실제 GGUF 모델이 없으면 직접 변환 필요.

   b. 테스트:
      llama-embedding \
        --model /opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf \
        --prompt "테스트 문장" \
        --embd-normalize 2

4. [Block D] 최종 환경 변수:
   ENV EMBEDDING_MODEL=/opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf \
       WAKE_WORD_MODEL=/opt/rover/models/wake_word/porcupine.ppn \
       PORCUPINE_LIB=/usr/local/lib/libpv_porcupine.so

5. [Block E] HEALTHCHECK 업데이트:
   HEALTHCHECK --interval=30s --timeout=15s --start-period=45s --retries=3 \
     CMD llama-cli --version && \
         python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'" && \
         python3 -c "import pvporcupine; print('Porcupine OK')" && \
         test -f /opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf

[Constraints]
- PyTorch는 변환 후 즉시 제거 (zero PyTorch in final image)
- Porcupine는 free tier 라이선스 (월 3000회 호출 제한)
- 임베딩 모델 VRAM < 300MB

[Alternative — Sentence-Transformers ONNX]
KoSimCSE GGUF가 없으면:
- sentence-transformers 설치 (pip3 install sentence-transformers)
- ONNX export 후 ONNX Runtime으로 추론
- 메모리 사용량 유사 (~300MB)

[Verification]
RUN test -f /opt/rover/models/embedding/kosimcse-roberta-q8_0.gguf && \
    test -f /usr/local/lib/libpv_porcupine.so && \
    llama-embedding --version

[Output]
- Dockerfile에 `FROM ocr-runtime AS full-runtime` 스테이지 추가
- 총 라인 수: ~150줄 추가 예상
- 최종 이미지 크기: 3.5~4.5GB (base 800MB + deps 2.7~3.7GB)
```

---

## 📦 Prompt D: SQLite 스키마 초기화 스크립트

```
[Context]
- SQLite 라이브러리는 이미 Dockerfile base 스테이지에 설치됨
- Goal: Create SQL initialization script with 8 tables from plan.md
- Output: Create /opt/rover/db/init_schema.sql inside container

[Task]
Create a SQL file that initializes the JetRover database schema.
This script will be executed when the container starts for the first time.

Requirements:

1. [File] /opt/rover/db/init_schema.sql:

   Content (based on plan.md section 5.2):

   ```sql
   -- ============================================
   -- JetRover Database Schema (SQLite)
   -- 버전: 3.0
   -- 초기화 날짜: 자동 생성
   -- ============================================

   -- Pragma settings for performance
   PRAGMA journal_mode=WAL;
   PRAGMA synchronous=NORMAL;
   PRAGMA cache_size=-64000;  -- 64MB cache
   PRAGMA temp_store=MEMORY;

   -- 1. 대화 기록 (Conversation History)
   CREATE TABLE IF NOT EXISTS conversations (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
       session_id TEXT NOT NULL,
       user_utterance TEXT NOT NULL,
       asr_confidence REAL,
       intent TEXT,
       llm_prompt TEXT,
       llm_response TEXT,
       executed_action TEXT,
       execution_status TEXT CHECK(execution_status IN ('success', 'failed', 'pending')),
       execution_time_ms INTEGER,
       tts_response TEXT,
       embedding_vector BLOB
   );

   CREATE INDEX IF NOT EXISTS idx_conv_timestamp ON conversations(timestamp);
   CREATE INDEX IF NOT EXISTS idx_conv_session ON conversations(session_id);
   CREATE INDEX IF NOT EXISTS idx_conv_intent ON conversations(intent);

   -- 2. 물체 지식 베이스 (Object Knowledge)
   CREATE TABLE IF NOT EXISTS objects (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       name_ko TEXT NOT NULL,
       name_en TEXT,
       category TEXT,
       color TEXT,
       shape TEXT,
       size_cm TEXT,
       weight_g INTEGER,
       graspable BOOLEAN DEFAULT 1,
       grasp_points TEXT,
       created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
       last_seen_at DATETIME,
       last_seen_location TEXT,
       detection_count INTEGER DEFAULT 0,
       description_embedding BLOB,
       vlm_description TEXT,
       UNIQUE(name_ko, color)
   );

   CREATE INDEX IF NOT EXISTS idx_obj_category ON objects(category);
   CREATE INDEX IF NOT EXISTS idx_obj_last_seen ON objects(last_seen_at);

   -- 3. 공간 지도 (Spatial Memory)
   CREATE TABLE IF NOT EXISTS locations (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       name TEXT NOT NULL UNIQUE,
       location_type TEXT CHECK(location_type IN ('furniture', 'area', 'point')),
       slam_x REAL NOT NULL,
       slam_y REAL NOT NULL,
       slam_theta REAL,
       map_id TEXT,
       created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
       updated_at DATETIME,
       min_x REAL,
       min_y REAL,
       max_x REAL,
       max_y REAL
   );

   -- R-Tree virtual table for spatial queries
   CREATE VIRTUAL TABLE IF NOT EXISTS locations_rtree USING rtree(
       id,
       min_x, max_x,
       min_y, max_y
   );

   -- 4. 물체-위치 관계 (Object-Location Relations)
   CREATE TABLE IF NOT EXISTS object_locations (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       object_id INTEGER REFERENCES objects(id),
       location_id INTEGER REFERENCES locations(id),
       relation_type TEXT CHECK(relation_type IN ('on', 'in', 'near', 'under')),
       confidence REAL DEFAULT 1.0,
       detected_at DATETIME DEFAULT CURRENT_TIMESTAMP,
       detected_by TEXT CHECK(detected_by IN ('vlm', 'user', 'inference')),
       UNIQUE(object_id, location_id, relation_type)
   );

   CREATE INDEX IF NOT EXISTS idx_objloc_object ON object_locations(object_id);
   CREATE INDEX IF NOT EXISTS idx_objloc_location ON object_locations(location_id);

   -- 5. 시스템 상태 로그 (System Telemetry)
   CREATE TABLE IF NOT EXISTS system_logs (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
       log_level TEXT CHECK(log_level IN ('DEBUG', 'INFO', 'WARN', 'ERROR')),
       component TEXT,
       message TEXT,
       metadata TEXT
   );

   CREATE INDEX IF NOT EXISTS idx_logs_timestamp ON system_logs(timestamp);
   CREATE INDEX IF NOT EXISTS idx_logs_level ON system_logs(log_level);
   CREATE INDEX IF NOT EXISTS idx_logs_component ON system_logs(component);

   -- 6. 사용자 프로필 (User Profiles)
   CREATE TABLE IF NOT EXISTS users (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       name TEXT,
       voice_embedding BLOB,
       preferred_speed REAL DEFAULT 0.5,
       created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
       last_interaction DATETIME
   );

   -- 7. 작업 큐 (Task Queue)
   CREATE TABLE IF NOT EXISTS task_queue (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       task_type TEXT NOT NULL,
       priority INTEGER DEFAULT 5,
       status TEXT CHECK(status IN ('pending', 'running', 'completed', 'failed', 'cancelled')),
       parameters TEXT,
       created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
       started_at DATETIME,
       completed_at DATETIME,
       error_message TEXT
   );

   CREATE INDEX IF NOT EXISTS idx_task_status ON task_queue(status);
   CREATE INDEX IF NOT EXISTS idx_task_priority ON task_queue(priority, created_at);

   -- 8. 벡터 검색 캐시 (Embedding Cache)
   CREATE TABLE IF NOT EXISTS embedding_cache (
       id INTEGER PRIMARY KEY AUTOINCREMENT,
       text_hash TEXT UNIQUE,
       text_content TEXT,
       embedding BLOB NOT NULL,
       model_version TEXT,
       created_at DATETIME DEFAULT CURRENT_TIMESTAMP
   );

   CREATE INDEX IF NOT EXISTS idx_embed_hash ON embedding_cache(text_hash);

   -- Insert default data
   INSERT OR IGNORE INTO users (id, name, preferred_speed)
   VALUES (1, 'Default User', 0.5);

   INSERT OR IGNORE INTO system_logs (log_level, component, message)
   VALUES ('INFO', 'database', 'Schema initialized successfully');

   -- Database version tracking
   CREATE TABLE IF NOT EXISTS schema_version (
       version TEXT PRIMARY KEY,
       applied_at DATETIME DEFAULT CURRENT_TIMESTAMP
   );

   INSERT OR REPLACE INTO schema_version (version) VALUES ('3.0');
   ```

2. [Dockerfile 추가] full-runtime 스테이지에 추가:

   RUN cat > /opt/rover/db/init_schema.sql <<'SQL'
   [위 SQL 내용 heredoc으로 삽입]
   SQL

3. [Entrypoint 스크립트] /opt/rover/scripts/docker-entrypoint.sh:

   ```bash
   #!/bin/bash
   set -e

   # Initialize database if not exists
   DB_FILE=/opt/rover/db/rover.db

   if [ ! -f "$DB_FILE" ]; then
       echo "Initializing database..."
       sqlite3 "$DB_FILE" < /opt/rover/db/init_schema.sql
       echo "Database initialized: $DB_FILE"
   else
       echo "Database already exists: $DB_FILE"
   fi

   # Execute CMD
   exec "$@"
   ```

   Dockerfile에 추가:
   COPY docker-entrypoint.sh /opt/rover/scripts/
   RUN chmod +x /opt/rover/scripts/docker-entrypoint.sh
   ENTRYPOINT ["/opt/rover/scripts/docker-entrypoint.sh"]
   CMD ["/bin/bash"]

4. [Verification Layer]:
   RUN sqlite3 /tmp/test.db < /opt/rover/db/init_schema.sql && \
       sqlite3 /tmp/test.db "SELECT name FROM sqlite_master WHERE type='table';" && \
       rm /tmp/test.db

[Constraints]
- SQLite WAL mode for concurrent reads
- R-Tree extension must be enabled (default in modern SQLite)
- No external dependencies (pure SQLite)

[Output]
- /opt/rover/db/init_schema.sql file created in container
- /opt/rover/scripts/docker-entrypoint.sh added
- ENTRYPOINT configured in Dockerfile
```

---

## 📦 Prompt E: docker-compose.yml 작성

```
[Context]
- Dockerfile is complete with all AI models and database schema
- Goal: Create docker-compose.yml for multi-container orchestration
- Architecture: Host-Brain separation (Brain = AI container, Host = ROS2 control)
- Output: Create docker-compose.yml in project root

[Reference]
See plan.md section 6.2 for architecture design.

[Task]
Write a production-ready docker-compose.yml file.

Requirements:

1. [File] docker-compose.yml (project root):

   ```yaml
   version: '3.8'

   services:
     # =========================================
     # Brain Core — Always-on AI orchestrator
     # =========================================
     brain_core:
       image: jetrover/brain:latest
       container_name: rover_brain_core
       runtime: nvidia
       restart: unless-stopped

       # OOM protection — medium priority
       mem_limit: 2200m
       mem_reservation: 1800m

       environment:
         - NVIDIA_VISIBLE_DEVICES=all
         - NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
         - ROS_DOMAIN_ID=42
         - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
         - FASTRTPS_DEFAULT_PROFILES_FILE=/opt/rover/config/fastdds.xml
         - CUDA_VISIBLE_DEVICES=0
         - ORT_TENSORRT_ENGINE_CACHE_ENABLE=1

       volumes:
         # Database persistence
         - ./db:/opt/rover/db
         # Model files (read-only)
         - ./models:/opt/rover/models:ro
         # Temporary files (shared with host)
         - /tmp:/tmp
         # ROS2 workspace source (development mode)
         - ./src/ai:/opt/rover/ws/src:ro

       network_mode: host

       # Healthcheck
       healthcheck:
         test: ["CMD-SHELL", "curl -f http://localhost:8080/health || exit 1"]
         interval: 30s
         timeout: 10s
         retries: 3
         start_period: 60s

       # OOM score adjustment (via docker run --oom-score-adj)
       # -500: protected from OOM killer, but not absolute
       sysctls:
         - net.core.rmem_max=2147483647
         - net.core.wmem_max=2147483647

       deploy:
         resources:
           reservations:
             devices:
               - driver: nvidia
                 count: 1
                 capabilities: [gpu]

     # =========================================
     # LLM Runner — Sacrificial service
     # =========================================
     llm_service:
       image: jetrover/brain:latest
       container_name: rover_llm
       runtime: nvidia
       restart: on-failure:3

       # OOM score: 800 (highest = killed first)
       mem_limit: 1800m
       oom_score_adj: 800

       environment:
         - NVIDIA_VISIBLE_DEVICES=all
         - ROS_DOMAIN_ID=42
         - LLM_MODEL=/opt/rover/models/llm/qwen2.5-3b-instruct-q4_k_m.gguf
         - LLAMA_SERVER_PORT=11434

       volumes:
         - ./models/llm:/opt/rover/models/llm:ro
         - /tmp:/tmp

       network_mode: host

       command: >
         llama-server
         --model /opt/rover/models/llm/qwen2.5-3b-instruct-q4_k_m.gguf
         --port 11434
         --ctx-size 2048
         --n-gpu-layers 99
         --threads 4
         --parallel 2

       healthcheck:
         test: ["CMD", "curl", "-f", "http://localhost:11434/health"]
         interval: 15s
         timeout: 5s
         retries: 2

       depends_on:
         - brain_core

       deploy:
         resources:
           limits:
             memory: 1800m
           reservations:
             devices:
               - driver: nvidia
                 count: 1
                 capabilities: [gpu]

     # =========================================
     # VLM Runner — Sacrificial service
     # =========================================
     vlm_service:
       image: jetrover/brain:latest
       container_name: rover_vlm
       runtime: nvidia
       restart: on-failure:3

       # OOM score: 700 (killed second)
       mem_limit: 2500m
       oom_score_adj: 700

       environment:
         - NVIDIA_VISIBLE_DEVICES=all
         - ROS_DOMAIN_ID=42
         - VLM_MODEL=/opt/rover/models/vlm/moondream2-q4_k_m.gguf
         - VLM_MMPROJ=/opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf

       volumes:
         - ./models/vlm:/opt/rover/models/vlm:ro
         - /tmp:/tmp

       network_mode: host

       command: >
         llama-server
         --model /opt/rover/models/vlm/moondream2-q4_k_m.gguf
         --mmproj /opt/rover/models/vlm/moondream2-mmproj-q4_0.gguf
         --port 8081
         --ctx-size 2048
         --n-gpu-layers 99

       healthcheck:
         test: ["CMD", "curl", "-f", "http://localhost:8081/health"]
         interval: 15s
         timeout: 5s
         retries: 2

       depends_on:
         - brain_core

       deploy:
         resources:
           limits:
             memory: 2500m
           reservations:
             devices:
               - driver: nvidia
                 count: 1
                 capabilities: [gpu]

     # =========================================
     # Host Bridge — Absolute protection
     # =========================================
     host_bridge:
       image: jetrover/brain:latest
       container_name: rover_host_bridge
       restart: always

       # OOM score: -1000 (never killed)
       mem_limit: 512m
       oom_score_adj: -1000

       environment:
         - ROS_DOMAIN_ID=42

       volumes:
         - /tmp:/tmp

       network_mode: host
       privileged: true

       command: >
         ros2 run rover_common heartbeat_node

       healthcheck:
         test: ["CMD", "ros2", "topic", "echo", "/heartbeat", "--once"]
         interval: 10s
         timeout: 3s
         retries: 5

   # =========================================
   # Volumes (optional persistent storage)
   # =========================================
   volumes:
     rover_db:
       driver: local
   ```

2. [.env 파일] docker-compose용 환경 변수:

   Create `.env` file in project root:
   ```
   # JetRover Docker Compose Configuration
   COMPOSE_PROJECT_NAME=jetrover

   # Image tags
   BRAIN_IMAGE_TAG=latest

   # Paths (absolute)
   ROVER_ROOT=/home/ubuntu/AI_secretary_robot
   MODELS_PATH=/home/ubuntu/AI_secretary_robot/models
   DB_PATH=/home/ubuntu/AI_secretary_robot/db

   # ROS2
   ROS_DOMAIN_ID=42

   # NVIDIA
   NVIDIA_VISIBLE_DEVICES=all
   ```

3. [Makefile] 편의 명령어:

   ```makefile
   .PHONY: up down restart logs build health

   up:
   	docker compose up -d

   down:
   	docker compose down

   restart:
   	docker compose restart

   logs:
   	docker compose logs -f --tail=100

   build:
   	docker build -t jetrover/brain:latest -f docker/Dockerfile .

   health:
   	@echo "=== Brain Core ==="
   	@docker exec rover_brain_core curl -s http://localhost:8080/health || echo "FAIL"
   	@echo "\n=== LLM Service ==="
   	@docker exec rover_llm curl -s http://localhost:11434/health || echo "FAIL"
   	@echo "\n=== VLM Service ==="
   	@docker exec rover_vlm curl -s http://localhost:8081/health || echo "FAIL"

   clean:
   	docker compose down -v
   	docker image prune -f
   ```

[Constraints]
- network_mode: host (ROS2 DDS 통신 필수)
- runtime: nvidia (모든 AI 서비스)
- OOM score adjustments require --privileged or specific capabilities
- Healthcheck endpoints must be implemented in each service

[Verification]
docker compose config  # Validate YAML syntax
docker compose up -d
docker compose ps
docker compose logs -f

[Output]
- docker-compose.yml in project root
- .env file in project root
- Makefile (optional) in project root
```

---

## 📦 Prompt F: Health Check & Heartbeat 시스템

```
[Context]
- docker-compose.yml defines healthcheck endpoints
- Goal: Implement /health FastAPI endpoint and /heartbeat ROS2 topic
- Output: Add health monitoring code to Dockerfile

[Task]
Add health monitoring system to the Docker image.

Requirements:

1. [Block A] FastAPI Health Server (/opt/rover/scripts/health_server.py):

   ```python
   #!/usr/bin/env python3
   """
   FastAPI Health Check Server for JetRover Brain Container
   Endpoints:
     GET /health — Overall system health
     GET /llm/status — LLM service status
     GET /vlm/status — VLM service status
     GET /metrics — Prometheus-compatible metrics
   """

   from fastapi import FastAPI
   from fastapi.responses import JSONResponse
   import psutil
   import subprocess
   import time

   app = FastAPI(title="JetRover Health API", version="1.0")

   START_TIME = time.time()

   def get_gpu_memory():
       """Query NVIDIA GPU memory using tegrastats"""
       try:
           result = subprocess.run(
               ['tegrastats', '--interval', '100'],
               capture_output=True,
               text=True,
               timeout=1
           )
           # Parse tegrastats output
           # Example: GR3D 25%@1300MHz EMC 45%@3200MHz RAM 3456/7680MB
           return {"status": "ok", "raw": result.stdout}
       except Exception as e:
           return {"status": "error", "message": str(e)}

   @app.get("/health")
   async def health_check():
       """Overall system health"""
       mem = psutil.virtual_memory()
       cpu = psutil.cpu_percent(interval=0.1)
       gpu = get_gpu_memory()

       return JSONResponse({
           "status": "ok" if mem.available > 2.5e9 else "degraded",
           "timestamp": time.time(),
           "uptime_seconds": time.time() - START_TIME,
           "memory": {
               "total_gb": round(mem.total / 1e9, 2),
               "available_gb": round(mem.available / 1e9, 2),
               "percent_used": mem.percent
           },
           "cpu_percent": cpu,
           "gpu": gpu
       })

   @app.get("/llm/status")
   async def llm_status():
       """Check if LLM server is responsive"""
       try:
           import requests
           resp = requests.get("http://localhost:11434/health", timeout=2)
           return {"status": "ok" if resp.status_code == 200 else "error"}
       except Exception as e:
           return {"status": "error", "message": str(e)}

   @app.get("/vlm/status")
   async def vlm_status():
       """Check if VLM server is responsive"""
       try:
           import requests
           resp = requests.get("http://localhost:8081/health", timeout=2)
           return {"status": "ok" if resp.status_code == 200 else "error"}
       except Exception as e:
           return {"status": "error", "message": str(e)}

   @app.get("/metrics")
   async def prometheus_metrics():
       """Prometheus-compatible metrics"""
       mem = psutil.virtual_memory()
       return f"""
   # HELP rover_memory_available_bytes Available memory in bytes
   # TYPE rover_memory_available_bytes gauge
   rover_memory_available_bytes {mem.available}

   # HELP rover_uptime_seconds Container uptime
   # TYPE rover_uptime_seconds counter
   rover_uptime_seconds {time.time() - START_TIME}
   """

   if __name__ == "__main__":
       import uvicorn
       uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")
   ```

   Install dependencies in Dockerfile:
   RUN pip3 install --no-cache-dir fastapi uvicorn psutil requests

2. [Block B] ROS2 Heartbeat Node (C++) (/opt/rover/src/heartbeat_node.cpp):

   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/header.hpp>
   #include <chrono>

   using namespace std::chrono_literals;

   class HeartbeatNode : public rclcpp::Node {
   public:
       HeartbeatNode() : Node("heartbeat_node") {
           publisher_ = this->create_publisher<std_msgs::msg::Header>("/heartbeat", 10);
           timer_ = this->create_wall_timer(
               1s, std::bind(&HeartbeatNode::publish_heartbeat, this));

           RCLCPP_INFO(this->get_logger(), "Heartbeat node started (1 Hz)");
       }

   private:
       void publish_heartbeat() {
           auto msg = std_msgs::msg::Header();
           msg.stamp = this->now();
           msg.frame_id = "brain_container";
           publisher_->publish(msg);
       }

       rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
       rclcpp::TimerBase::SharedPtr timer_;
   };

   int main(int argc, char** argv) {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<HeartbeatNode>();
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

   Build in Dockerfile:
   RUN cd /opt/rover/heartbeat_ws && \
       cmake -B build -DCMAKE_PREFIX_PATH=/opt/ros/humble && \
       cmake --build build --target heartbeat_node && \
       install -m 0755 build/heartbeat_node /usr/local/bin/

3. [Block C] Supervisor 설정 (선택적):

   Multi-process 관리를 위해 supervisord 사용:

   Install:
   RUN apt-get update && apt-get install -y --no-install-recommends supervisor && \
       rm -rf /var/lib/apt/lists/*

   Config (/etc/supervisor/conf.d/jetrover.conf):
   ```ini
   [supervisord]
   nodaemon=true
   logfile=/var/log/supervisor/supervisord.log
   pidfile=/var/run/supervisord.pid

   [program:health_server]
   command=python3 /opt/rover/scripts/health_server.py
   autostart=true
   autorestart=true
   stdout_logfile=/var/log/supervisor/health_server.log
   stderr_logfile=/var/log/supervisor/health_server_err.log

   [program:heartbeat_node]
   command=/bin/bash -c "source /opt/ros/humble/setup.bash && /usr/local/bin/heartbeat_node"
   autostart=true
   autorestart=true
   stdout_logfile=/var/log/supervisor/heartbeat.log
   stderr_logfile=/var/log/supervisor/heartbeat_err.log
   ```

   Dockerfile CMD:
   CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]

4. [Block D] Dockerfile HEALTHCHECK 업데이트:

   HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
     CMD curl -f http://localhost:8080/health || exit 1

[Constraints]
- Health server must start before ROS2 nodes
- /heartbeat topic must publish at exactly 1 Hz
- /health endpoint timeout: 200ms max

[Verification]
# In container:
curl http://localhost:8080/health
ros2 topic hz /heartbeat  # Should show ~1.0 Hz

[Output]
- /opt/rover/scripts/health_server.py
- /opt/rover/src/heartbeat_node.cpp (compiled binary)
- /etc/supervisor/conf.d/jetrover.conf (optional)
- Updated Dockerfile HEALTHCHECK
```

---

## 📦 Prompt G: 빌드 & 실행 자동화 스크립트

```
[Context]
- Dockerfile is complete
- docker-compose.yml is ready
- Goal: Create automation scripts for building, testing, and deploying
- Output: Shell scripts in /scripts directory

[Task]
Create build and deployment automation scripts.

Requirements:

1. [File] scripts/build_docker.sh:

   ```bash
   #!/bin/bash
   # JetRover Docker Image Build Script
   # Usage: ./scripts/build_docker.sh [--no-cache] [--stage STAGE]

   set -e

   PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
   cd "$PROJECT_ROOT"

   IMAGE_NAME="jetrover/brain"
   TAG="latest"
   DOCKERFILE="docker/Dockerfile"

   # Parse arguments
   BUILD_ARGS=""
   TARGET_STAGE=""

   while [[ $# -gt 0 ]]; do
       case $1 in
           --no-cache)
               BUILD_ARGS="$BUILD_ARGS --no-cache"
               shift
               ;;
           --stage)
               TARGET_STAGE="$2"
               shift 2
               ;;
           *)
               echo "Unknown option: $1"
               exit 1
               ;;
       esac
   done

   if [ -n "$TARGET_STAGE" ]; then
       BUILD_ARGS="$BUILD_ARGS --target $TARGET_STAGE"
   fi

   echo "========================================="
   echo "Building JetRover Docker Image"
   echo "========================================="
   echo "Image: $IMAGE_NAME:$TAG"
   echo "Dockerfile: $DOCKERFILE"
   echo "Target stage: ${TARGET_STAGE:-full-runtime (final)}"
   echo "Build args: $BUILD_ARGS"
   echo ""

   # Build
   docker build \
       -t "$IMAGE_NAME:$TAG" \
       -f "$DOCKERFILE" \
       $BUILD_ARGS \
       .

   echo ""
   echo "========================================="
   echo "Build Complete!"
   echo "========================================="
   docker images "$IMAGE_NAME:$TAG"

   echo ""
   echo "Run with: docker run --runtime=nvidia -it $IMAGE_NAME:$TAG"
   echo "Or use docker-compose: docker compose up -d"
   ```

2. [File] scripts/test_docker.sh:

   ```bash
   #!/bin/bash
   # JetRover Docker Image Test Script

   set -e

   IMAGE_NAME="jetrover/brain:latest"

   echo "Testing Docker image: $IMAGE_NAME"
   echo ""

   # Test 1: Image exists
   echo "[1/8] Checking image exists..."
   docker image inspect "$IMAGE_NAME" > /dev/null 2>&1
   echo "✓ Image found"

   # Test 2: ROS2 availability
   echo "[2/8] Testing ROS2..."
   docker run --rm "$IMAGE_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 --version"
   echo "✓ ROS2 OK"

   # Test 3: ONNX Runtime GPU
   echo "[3/8] Testing ONNX Runtime GPU..."
   docker run --rm --runtime=nvidia "$IMAGE_NAME" \
       python3 -c "import onnxruntime as ort; assert ort.get_device()=='GPU'; print('✓ ONNX Runtime GPU OK')"

   # Test 4: llama.cpp
   echo "[4/8] Testing llama.cpp..."
   docker run --rm "$IMAGE_NAME" llama-cli --version
   echo "✓ llama.cpp OK"

   # Test 5: Piper TTS
   echo "[5/8] Testing Piper TTS..."
   docker run --rm "$IMAGE_NAME" piper --version
   echo "✓ Piper TTS OK"

   # Test 6: Database schema
   echo "[6/8] Testing database schema..."
   docker run --rm "$IMAGE_NAME" bash -c "test -f /opt/rover/db/init_schema.sql && echo '✓ Schema file exists'"

   # Test 7: YOLO node binary
   echo "[7/8] Testing YOLO node..."
   docker run --rm "$IMAGE_NAME" bash -c "test -f /usr/local/bin/yolo_ros_node && echo '✓ YOLO node exists'"

   # Test 8: VLM model
   echo "[8/8] Testing VLM model..."
   docker run --rm "$IMAGE_NAME" bash -c "test -f /opt/rover/models/vlm/moondream2-q4_k_m.gguf && echo '✓ VLM model exists'"

   echo ""
   echo "========================================="
   echo "All tests passed!"
   echo "========================================="
   ```

3. [File] scripts/start_system.sh:

   ```bash
   #!/bin/bash
   # JetRover Complete System Startup Script
   # Follows plan.md section 6.4 execution sequence

   set -e

   PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
   cd "$PROJECT_ROOT"

   echo "========================================="
   echo "JetRover System Startup"
   echo "========================================="

   # Step 1: Jetson power mode
   echo "[1/7] Setting Jetson power mode (MAXN)..."
   sudo nvpmodel -m 2 || echo "⚠️ nvpmodel failed (ignore if not on Jetson)"
   nvpmodel -q || true

   # Step 2: Fan control (optional)
   echo "[2/7] Setting fan speed to max..."
   FAN_PWM="/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1"
   if ls $FAN_PWM 1> /dev/null 2>&1; then
       echo 255 | sudo tee $FAN_PWM > /dev/null
       echo "✓ Fan set to 100%"
   else
       echo "⚠️ Fan control not available"
   fi

   # Step 3: ZRAM (optional)
   echo "[3/7] Checking ZRAM..."
   swapon -s | grep zram || echo "⚠️ ZRAM not configured"

   # Step 4: Check models exist
   echo "[4/7] Verifying model files..."
   MODELS_DIR="$PROJECT_ROOT/models"

   check_model() {
       if [ -f "$1" ]; then
           echo "✓ $2"
       else
           echo "❌ Missing: $1"
           return 1
       fi
   }

   check_model "$MODELS_DIR/llm/qwen2.5-3b-instruct-q4_k_m.gguf" "LLM model"
   # check_model "$MODELS_DIR/vlm/moondream2-q4_k_m.gguf" "VLM model"
   # check_model "$MODELS_DIR/vision/yolo11n_fp16.engine" "YOLO engine"

   # Step 5: Start Docker containers
   echo "[5/7] Starting Docker containers..."
   docker compose up -d

   # Wait for containers to start
   sleep 10

   # Step 6: Health check
   echo "[6/7] Running health checks..."
   docker compose ps

   echo ""
   echo "Checking Brain Core health..."
   curl -f http://localhost:8080/health || echo "⚠️ Health check failed"

   # Step 7: Verify ROS2 heartbeat
   echo "[7/7] Verifying ROS2 heartbeat..."
   timeout 5 ros2 topic echo /heartbeat --once || echo "⚠️ Heartbeat not detected"

   echo ""
   echo "========================================="
   echo "System Startup Complete!"
   echo "========================================="
   echo ""
   echo "Monitor logs: docker compose logs -f"
   echo "Stop system: docker compose down"
   echo "Health check: curl http://localhost:8080/health"
   ```

4. [File] scripts/stop_system.sh:

   ```bash
   #!/bin/bash
   # JetRover System Shutdown Script

   set -e

   PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
   cd "$PROJECT_ROOT"

   echo "Stopping JetRover system..."

   docker compose down

   echo "System stopped."
   echo ""
   echo "Database is preserved in: db/rover.db"
   echo "Restart with: ./scripts/start_system.sh"
   ```

5. [File] scripts/logs.sh:

   ```bash
   #!/bin/bash
   # Tail logs for specific service or all

   SERVICE="${1:-}"

   cd "$(dirname "${BASH_SOURCE[0]}")/.."

   if [ -n "$SERVICE" ]; then
       docker compose logs -f --tail=100 "$SERVICE"
   else
       docker compose logs -f --tail=100
   fi
   ```

6. Make scripts executable:

   chmod +x scripts/*.sh

[Output]
- scripts/build_docker.sh
- scripts/test_docker.sh
- scripts/start_system.sh
- scripts/stop_system.sh
- scripts/logs.sh
- All scripts executable

[Verification]
./scripts/build_docker.sh --stage base  # Test build
./scripts/test_docker.sh  # Run tests
```

---

## 🔧 실행 순서 요약

1. **Prompt A**: VLM Stage 추가 → `docker/Dockerfile` 수정
2. **Prompt B**: OCR Stage 추가 → `docker/Dockerfile` 수정
3. **Prompt C**: 임베딩 & Wake Word Stage 추가 → `docker/Dockerfile` 완성
4. **Prompt D**: SQLite 스키마 추가 → `docker/Dockerfile`에 SQL heredoc 삽입
5. **Prompt E**: `docker-compose.yml`, `.env`, `Makefile` 작성
6. **Prompt F**: Health Check 시스템 추가 → `docker/Dockerfile` 수정
7. **Prompt G**: 자동화 스크립트 작성 → `scripts/` 디렉토리

**최종 빌드 & 실행:**
```bash
./scripts/build_docker.sh
./scripts/test_docker.sh
./scripts/start_system.sh
```

---

## 📝 주의사항

### 모델 파일 준비
Prompt 실행 전에 다음 모델들을 사전 다운로드/변환해야 합니다:

1. **VLM**: moondream2 GGUF 모델
   - HuggingFace에서 검색하거나 직접 변환

2. **임베딩**: KoSimCSE GGUF
   - llama.cpp convert 스크립트 사용

3. **YOLO**: TensorRT 엔진
   - Host에서 `trtexec`로 생성:
     ```bash
     trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_fp16.engine --fp16
     ```

### 디스크 공간
최종 이미지 크기: **약 4~5GB**
- base: 800MB
- ai-runtime: +1.5GB
- vision-runtime: +500MB
- vlm-runtime: +1GB (모델 번들 시)
- ocr-runtime: +300MB
- full-runtime: +400MB

### 메모리 요구사항
런타임 메모리 (8GB Jetson 기준):
- System: 1GB
- ROS2 Host: 1.5GB
- Brain Container: 2.2GB
- LLM Service: 1.8GB
- VLM Service: 2GB
- **여유: 0.5GB** (OOM 방지)

Multi-container 동시 실행 시 메모리 부족 가능 → OOM score 정책으로 우선순위 관리.
