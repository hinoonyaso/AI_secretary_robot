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
