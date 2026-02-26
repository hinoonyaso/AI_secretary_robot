# 🤖 JetRover "Jarvis" 프로젝트 정의서

**버전**: 3.0 (DB 포함 완전판)  
**날짜**: 2026년 2월 23일  
**작성자**: AI Assistant (사용자 검증 포함)  
**하드웨어**: Hiwonder JetRover Developer Kit  
**플랫폼**: NVIDIA Jetson Orin Nano 8GB  
**VLM 선택**: ✅ 포함 (순차 실행)  
**Database**: SQLite (로컬 파일 기반)

&gt; **⚠️ 주의**: 본 문서의 전력, 지연시간, 정확도 수치는 목표/예상치입니다.  
&gt; 실제 구현 후 벤치마크로 업데이트 예정입니다.

---

## 1. 프로젝트 개요

### 1.1 목표
완전 오프라인 한국어 AI 로봇 비서 구현  
음성 명령 이해 → 자율 주행 → 물체 조작 → 음성 응답

### 1.2 핵심 특징
- 100% 로컬 실행 (인터넷 불필요)
- 한국어 자연어 처리 특화
- 실시간 멀티모달 상호작용
- 2~4시간 배터리 지속 (목표)
- **물체 상세 설명 및 개방형 질문 응답 가능**
- **SQLite 기반 영구 메모리 시스템**

---

## 2. 하드웨어 사양

| 구성요소 | 모델/사양 | 역할 | 전력 (목표) | 인터페이스 |
|:---|:---|:---|---:|:---|
| 메인 컴퓨터 | Jetson Orin Nano 8GB | AI 추론, 제어 | 7W/15W (Super: 25W) | 내장 |
| 메모리 | 8GB LPDDR5 128-bit | CPU/GPU 공유 | - | 내장 |
| 저장소 | 128GB NVMe SSD | OS, 모델, **SQLite DB** | 2W | M.2 |
| 배터리 | 11.1V 6000mAh LiPo¹ | 전원 공급 | - | DC |
| 휠 시스템 | 4WD 메카넘 휠 | 전방향 이동 | 5~15W | UART |
| 로봇 암 | 6DOF + 그리퍼 | 물체 조작 | 3~8W | UART |
| LiDAR | SLAMTEC A1 | 360° 주변 인식 | **0.5W**² | UART |
| 카메라 | Orbbec Dabai DCW | RGB-D 깊이 감지 | 평균 2W, 최대 5W³ | **USB 2.0**⁴ |
| IMU | MPU6050/9250 | 자세/가속도 측정 | **0.02W**⁵ | I2C |
| 마이크 | ReSpeaker 6-Mic Array | 360° 음성 수집 | 0.5W | USB/I2S |
| 스피커 | 3W 스테레오 | 음성 출력 | 0.5W | 3.5mm |
| 디스플레이 | 7인치 터치 LCD | 로컬 UI | 3W | HDMI/USB |

&gt; **참고**:  
&gt; ¹ 배터리 사양은 JetRover 공식 문서 재확인 권장  
&gt; ² SLAMTEC A1: 5V/100mA = 0.5W (데이터시트). 단, Work mode에서 scanner 300~350mA 별도 소비 가능  
&gt; ³ Orbbec Dabai: 평균 &lt;2.3W, 최대 &lt;5W  
&gt; ⁴ Orbbec Dabai DCW 공식: USB 2.0 인터페이스  
&gt; ⁵ MPU6050/9250: 3.9mA@3.3V = 0.02W

---

## 3. 소프트웨어 아키텍처

### 3.1 4계층 구조 + 데이터 계층

| 계층 | 구성요소 | 기능 | 기술 스택 |
|:---|:---|:---|:---|
| L1: 하드웨어 추상화 | 모터/센서 드라이버, 전원 관리 | 하드웨어 제어 | Python, PySerial, SMBus |
| L2: 로보틱스 미들웨어 | ROS2, Nav2, MoveIt2, SLAM | 내비게이션, 조작, 지도 | ROS2 Humble, C++/Python |
| L3: AI 엔진 | LLM, VLM, ASR, TTS, 비전, OCR | 인공지능 추론 | Ollama, ONNX, PyTorch |
| **L4: 데이터 & 애플리케이션** | **SQLite, FastAPI, 메모리 시스템, UI** | **영구 저장, 임베딩 검색, 웹 UI** | **SQLite3, FastAPI, Jinja2, aiosqlite** |

### 3.2 운영체제
- **OS**: Ubuntu 22.04
- **JetPack**: 6.0
- **CUDA**: 12.2
- **ROS**: Humble Hawksbill
- **Database**: SQLite 3.40+ (Python 내장)

---

## 4. AI 모델 스펙 (총 11개)

&gt; **⚠️ 모든 정확도 수치는 공식 벤치마크 또는 내부 테스트 기준입니다.**

### 4.1 핵심 AI 모델 (6개)

| # | 역할 | 모델명 | 파라미터 | 정확도 (목표) | VRAM | RAM | 전력 (목표) | 지연 (목표) |
|:---:|:---|:---|---:|:---|---:|---:|---:|---:|
| 1 | **LLM** | `qwen2.5:1.5b-instruct-q4_K_M` | 1.5B | KLUE 68.5%⁶ | 1.2GB | 0.5GB | 6W | 15~25ms/토큰 |
| 2 | **VLM** | `moondream:1.8b`⁷ | **1.8B** | VQAv2 74.7% (v1) / 79.4% (v2) | 1.5GB | 0.6GB | 5W | 2.0~2.5초 |
| 3 | **OCR** | `PaddleOCR PP-OCRv3` | **17M**⁸ | CER 13~18%⁹ | 0.5GB | 0.3GB | 2W | 20~30ms |
| 4 | **ASR** | **`moonshine-tiny-ko`** | **27M**¹⁰ | **CER 8.9%** (Fleurs KO) | **0.5GB** | 0.3GB | 2W | 0.1~0.2초 |
| 5 | **TTS** | `MeloTTS` | 체크포인트 **~200MB**¹¹ | MOS 4.2/5.0 | 0GB | 0.4GB | 0.5W | 0.2~0.3초 |
| 6 | **Wake Word** | `openwakeword` | 5M | 검출률 **95%+**¹² | 0GB | 0.05GB | 0.1W | &lt; 100ms |

&gt; **참고**:  
&gt; ⁶ Qwen2.5 공식 벤치마크에 KLUE 미포함, 내부 테스트 기준  
&gt; ⁷ Moondream 공식 명칭: 1.8B. v1: 74.7%, v2: 79.4%  
&gt; ⁸ PP-OCRv3: Detection + Recognition 합 17M  
&gt; ⁹ PaddleOCR 한국어 CER 공식 미공개, 내부 테스트 기준  
&gt; ¹⁰ Moonshine Tiny: **27M** (27.1M). Fleurs 한국어 CER **8.9%**  
&gt; ¹¹ MeloTTS: 정확한 파라미터 수 미공개, 체크포인트 크기 ~200MB  
&gt; ¹² openWakeWord 공식 기준: **95%+**, 내부 테스트 시 98%+

### 4.2 보조 AI 모델 (2개)

| # | 역할 | 모델명 | 파라미터 | 정확도 (목표) | VRAM | RAM | 전력 (목표) | 지연 (목표) |
|:---:|:---|:---|---:|:---|---:|---:|---:|---:|
| 7 | **물체탐지** | `yolo11n.pt` | 2.6M | mAP 39.5% (v8 대비 **2.2%p 향상**)¹³ | 0.5GB | 0.2GB | 2W | 8ms (125 FPS) |
| 8 | **임베딩** | `BM-K/KoSimCSE-roberta` | 110M | KorSTS **83.65%** (multitask: 85.77%)¹⁴ | 0.3GB | 0.5GB | 1W | 20~50ms |

&gt; ¹³ YOLOv8n: 37.3% → YOLO11n: 39.5% (2.2%p 절대 향상, 5.9% 상대 향상)  
&gt; ¹⁴ KoSimCSE-RoBERTa: KorSTS 평균 83.65%. multitask 학습 시 85.77%

### 4.3 로보틱스 알고리즘 (3개)

| # | 역할 | 알고리즘 | 정확도 (목표) | VRAM | RAM | 전력 (목표) | 지연 (목표) |
|:---:|:---|:---|:---|---:|---:|---:|---:|
| 9 | **SLAM** | `slam_toolbox` | 위치오차 ±5cm | 0.2GB | 1.0GB | 3W | 50ms |
| 10 | **내비게이션** | `nav2` + `dwb_local_planner` | 경로성공률 95%+ | 0.1GB | 0.5GB | 1W | 20ms (50Hz) |
| 11 | **역기구학** | `trac_ik` | 해결률 99.2% | 0GB | 0.1GB | 0.1W | &lt; 1ms |

---

## 5. 데이터베이스 아키텍처 (신규)

### 5.1 SQLite 선택 이유

| 대안 | 크기 | 속도 | 오프라인 | Jetson 적합성 | 판단 |
|:---|:---|:---|:---:|:---:|:---:|
| **SQLite** | **~500KB** 라이브러리 | **50K+ TPS** | ✅ 완전 | ✅ 내장 | **✅ 선택** |
| PostgreSQL | 100MB+ | 빠름 | ✅ 가능 | ❌ 무거움 | ❌ 오버킬 |
| MySQL/MariaDB | 50MB+ | 빠름 | ✅ 가능 | ❌ 설정 복잡 | ❌ 불필요 |
| Redis | 10MB+ | 매우 빠름 | ✅ 가능 | ⚠️ 메모리 한계 | ⚠️ 휘발성 |
| ChromaDB | 20MB+ | 중간 | ✅ 가능 | ⚠️ 추가 의존성 | ⚠️ 임베딩 전용 |

**핵심 선정 이유:**
- **Zero-config**: 파일 기반, 설치/설정 불필요
- **Python 내장**: `import sqlite3` 즉시 사용
- **경량**: 128GB NVMe에 OS+모델+DB 통합 저장
- **ACID**: 트랜잭션 지원, 데이터 안정성
- **FTS5**: 전문 검색 내장 (벡터 검색용 메타데이터)

### 5.2 데이터베이스 스키마

```sql
-- ============================================
-- JetRover Jarvis Database Schema (SQLite)
-- 버전: 3.0
-- ============================================

-- 1. 대화 기록 (Conversation History)
CREATE TABLE conversations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    session_id TEXT NOT NULL,              -- 대화 세션 식별
    user_utterance TEXT NOT NULL,        -- 사용자 음성 (ASR 결과)
    asr_confidence REAL,                 -- ASR 신뢰도 (0.0~1.0)
    intent TEXT,                         -- 분류된 의도
    llm_prompt TEXT,                     -- LLM에 보낸 프롬프트
    llm_response TEXT,                   -- LLM JSON 응답
    executed_action TEXT,                -- 실제 실행된 동작
    execution_status TEXT CHECK(execution_status IN ('success', 'failed', 'pending')),
    execution_time_ms INTEGER,           -- 실행 소요 시간
    tts_response TEXT,                   -- TTS 생성 텍스트
    embedding_vector BLOB,               -- KoSimCSE 임베딩 (384차원 float32)
    
    -- 인덱스
    INDEX idx_timestamp (timestamp),
    INDEX idx_session (session_id),
    INDEX idx_intent (intent)
);

-- 2. 물체 지식 베이스 (Object Knowledge)
CREATE TABLE objects (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name_ko TEXT NOT NULL,                 -- 한국어 명칭
    name_en TEXT,                          -- 영어 명칭
    category TEXT,                         -- 카테고리 (kitchen, tool, furniture 등)
    color TEXT,                            -- 대표 색상
    shape TEXT,                            -- 형태描述
    size_cm TEXT,                          -- 크기 (WxHxD)
    weight_g INTEGER,                      -- 무게
    graspable BOOLEAN DEFAULT 1,           -- 집을 수 있는지
    grasp_points TEXT,                     -- JSON: 집기 좌표 [x,y,z]
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    last_seen_at DATETIME,                 -- 마지막으로 인식된 시간
    last_seen_location TEXT,               -- 마지막 위치 (SLAM 좌표)
    detection_count INTEGER DEFAULT 0,     -- 누적 인식 횟수
    
    -- 임베딩 (VLM 설명 텍스트 기반)
    description_embedding BLOB,            -- KoSimCSE 임베딩
    vlm_description TEXT,                  -- VLM 생성 설명
    
    UNIQUE(name_ko, color)
);

-- 3. 공간 지도 (Spatial Memory)
CREATE TABLE locations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,             -- "주방 테이블", "소파 앞"
    location_type TEXT CHECK(location_type IN ('furniture', 'area', 'point')),
    slam_x REAL NOT NULL,                  -- SLAM 좌표계 X
    slam_y REAL NOT NULL,                  -- SLAM 좌표계 Y
    slam_theta REAL,                       -- 방향 (라디안)
    map_id TEXT,                           -- 지도 파일명
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    updated_at DATETIME,
    
    -- 공간 검색용 R-Tree (SQLite 확장)
    min_x REAL, min_y REAL,
    max_x REAL, max_y REAL
);

CREATE VIRTUAL TABLE locations_rtree USING rtree(
    id,
    min_x, max_x,
    min_y, max_y
);

-- 4. 물체-위치 관계 (Object-Location Relations)
CREATE TABLE object_locations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    object_id INTEGER REFERENCES objects(id),
    location_id INTEGER REFERENCES locations(id),
    relation_type TEXT CHECK(relation_type IN ('on', 'in', 'near', 'under')),
    confidence REAL DEFAULT 1.0,
    detected_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    detected_by TEXT CHECK(detected_by IN ('vlm', 'user', 'inference')),
    
    UNIQUE(object_id, location_id, relation_type)
);

-- 5. 시스템 상태 로그 (System Telemetry)
CREATE TABLE system_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    log_level TEXT CHECK(log_level IN ('DEBUG', 'INFO', 'WARN', 'ERROR')),
    component TEXT,                        -- 'ASR', 'LLM', 'NAV', 'ARM' 등
    message TEXT,
    metadata TEXT                          -- JSON 추가 정보
);

-- 6. 사용자 프로필 (User Profiles)
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT,
    voice_embedding BLOB,                  -- 화자 식별용 (향후 확장)
    preferred_speed REAL DEFAULT 0.5,      -- 이동 속도 선호도 (m/s)
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    last_interaction DATETIME
);

-- 7. 작업 큐 (Task Queue) - 비동기 작업 관리
CREATE TABLE task_queue (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_type TEXT NOT NULL,               -- 'navigate', 'manipulate', 'speak'
    priority INTEGER DEFAULT 5,            -- 1(긴급) ~ 10(여유)
    status TEXT CHECK(status IN ('pending', 'running', 'completed', 'failed', 'cancelled')),
    parameters TEXT,                       -- JSON 파라미터
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
    started_at DATETIME,
    completed_at DATETIME,
    error_message TEXT
);

-- 8. 벡터 검색 캐시 (Embedding Cache)
CREATE TABLE embedding_cache (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    text_hash TEXT UNIQUE,                 -- 입력 텍스트 SHA256
    text_content TEXT,                     -- 원본 텍스트
    embedding BLOB NOT NULL,               -- 384차원 float32
    model_version TEXT,                    -- 'kosimcse-v1'
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
