/**
 * ADAS HUD Dashboard - Core Logic & Navigation Canvas Renderer
 */

// State Management
let appState = {
    mode: "OFFLINE", // "OFFLINE", "REPLAY", "LIVE"
    isPlaying: false,
    currentFrameIndex: 0,
    playbackSpeed: 1.0,
    audioEnabled: true,

    // Config Parameters (updated from stream/replay)
    egoSpeed: 10.0, // m/s
    safetyRadius: 1.0, // m
    acceleratorPressed: false,
    acceleratorPedal: 0.0,
    brakePressed: false,
    brakeLights: false,

    // Animation/Simulation variables
    roadOffset: 0,
    lastAnimTime: 0,

    // Data Buffers
    frames: [], // Replay Mode: combined frames
    latestLiveFrame: null,
    liveStreamSource: null,

    // Audio Context for direct warning synthesis
    audioCtx: null,
    lastAudioWarningTime: 0,
    activeWarningSound: 0 // 0: None, 1: Level 1, 2: Level 2, 3: Level 3
};

// UI Elements DOM References
const el = {
    emergencyGlow: document.getElementById("emergency-glow"),
    hdrFrameId: document.getElementById("hdr-frame-id"),
    hdrTimestamp: document.getElementById("hdr-timestamp"),
    systemStatus: document.getElementById("system-status"),
    instSpeed: document.getElementById("inst-speed"),
    instSpeedKmh: document.getElementById("inst-speed-kmh"),
    pedalAccel: document.getElementById("pedal-accel"),
    pedalBrake: document.getElementById("pedal-brake"),
    instAccelState: document.getElementById("inst-accel-state"),
    instAccelValue: document.getElementById("inst-accel-value"),
    instBrakeState: document.getElementById("inst-brake-state"),
    instBrakeValue: document.getElementById("inst-brake-value"),
    instDecel: document.getElementById("inst-decel"),
    decelBar: document.getElementById("decel-bar"),
    warningBanner: document.getElementById("warning-banner"),
    warnTitle: document.getElementById("warn-title"),
    warnAction: document.getElementById("warn-action"),
    paramEgoSpeed: document.getElementById("param-ego-speed"),
    paramSafetyRadius: document.getElementById("param-safety-radius"),
    btnAudioToggle: document.getElementById("btn-audio-toggle"),
    radarCanvas: document.getElementById("radar-canvas"),
    radarContainer: document.getElementById("radar-container"),
    trackList: document.getElementById("track-list"),

    // Playback
    btnPlayPause: document.getElementById("btn-play-pause"),
    btnStop: document.getElementById("btn-stop"),
    timelineSlider: document.getElementById("timeline-slider"),
    timeCurrent: document.getElementById("time-current"),
    timeTotal: document.getElementById("time-total"),
    speedSelect: document.getElementById("speed-select"),
    dropZone: document.getElementById("drop-zone"),
    btnBrowse: document.getElementById("btn-browse"),
    fileSelector: document.getElementById("file-selector"),
    loadedStatus: document.getElementById("loaded-status"),

    // Latency
    latPerception: document.getElementById("lat-perception"),
    latValPerception: document.getElementById("lat-val-perception"),
    latPrediction: document.getElementById("lat-prediction"),
    latValPrediction: document.getElementById("lat-val-prediction"),
    latTracking: document.getElementById("lat-tracking"),
    latValTracking: document.getElementById("lat-val-tracking"),
    latTotal: document.getElementById("lat-total")
};

// Canvas context
let ctx = el.radarCanvas.getContext("2d");

// Scale mapping configuration
const RADAR_SCALE = 12; // 1 meter = 12 pixels
let canvasCenter = { x: 0, y: 0 };
let egoCarPos = { x: 0, y: 0 };

// Initialize App
window.addEventListener("DOMContentLoaded", () => {
    initCanvas();
    setupEventListeners();
    setupLiveSSE();

    // Run drawing loop
    appState.lastAnimTime = performance.now();
    requestAnimationFrame(animationLoop);
});

// Canvas Sizing
function initCanvas() {
    const rect = el.radarContainer.getBoundingClientRect();
    el.radarCanvas.width = rect.width;
    el.radarCanvas.height = rect.height;

    canvasCenter.x = el.radarCanvas.width / 2;
    canvasCenter.y = el.radarCanvas.height / 2;

    // Ego car fixed at bottom center
    egoCarPos.x = canvasCenter.x;
    egoCarPos.y = el.radarCanvas.height - 120;
}

window.addEventListener("resize", initCanvas);

// Audio Synthesis using Web Audio API (Zero external file dependencies)
function initAudio() {
    if (!appState.audioCtx) {
        appState.audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    }
}

function playWarningBeep(level) {
    if (!appState.audioEnabled) return;
    initAudio();

    const now = appState.audioCtx.currentTime;
    // Throttle audio triggers to avoid overlapping beeps
    const throttleTime = level === 3 ? 150 : (level === 2 ? 400 : 800);
    const timeSinceLast = Date.now() - appState.lastAudioWarningTime;
    if (timeSinceLast < throttleTime) return;

    appState.lastAudioWarningTime = Date.now();

    if (level === 1) {
        // LEVEL 1: Gentle alert, double chime
        beep(587.33, 0.08, 0.1); // D5
        setTimeout(() => beep(587.33, 0.08, 0.1), 120);
    } else if (level === 2) {
        // LEVEL 2: Staccato caution beep
        beep(880, 0.12, 0.25); // A5
    } else if (level === 3) {
        // LEVEL 3: High alarm emergency siren
        beep(1046.50, 0.06, 0.4); // C6
        setTimeout(() => beep(1396.91, 0.06, 0.4), 80); // F6
    }
}

function beep(frequency, duration, volume) {
    try {
        if (!appState.audioCtx || appState.audioCtx.state === "suspended") return;

        let osc = appState.audioCtx.createOscillator();
        let gain = appState.audioCtx.createGain();

        osc.type = "sine";
        osc.frequency.setValueAtTime(frequency, appState.audioCtx.currentTime);

        gain.gain.setValueAtTime(volume, appState.audioCtx.currentTime);
        gain.gain.exponentialRampToValueAtTime(0.0001, appState.audioCtx.currentTime + duration);

        osc.connect(gain);
        gain.connect(appState.audioCtx.destination);

        osc.start();
        osc.stop(appState.audioCtx.currentTime + duration);
    } catch (e) {
        console.error("Audio beep error:", e);
    }
}

// Setup static and dynamic interaction handlers
function setupEventListeners() {
    // Audio toggling
    el.btnAudioToggle.addEventListener("click", () => {
        appState.audioEnabled = !appState.audioEnabled;
        if (appState.audioEnabled) {
            el.btnAudioToggle.textContent = "🔊 AUDIO: ON";
            el.btnAudioToggle.className = "btn btn-secondary btn-audio-on";
            initAudio();
        } else {
            el.btnAudioToggle.textContent = "🔇 AUDIO: OFF";
            el.btnAudioToggle.className = "btn btn-secondary btn-audio-off";
        }
    });

    // File selection UI trigger
    el.btnBrowse.addEventListener("click", () => el.fileSelector.click());
    el.fileSelector.addEventListener("change", handleFileSelection);

    // Timeline Slider Change
    el.timelineSlider.addEventListener("input", (e) => {
        const frameIdx = parseInt(e.target.value);
        if (appState.frames.length > 0 && frameIdx < appState.frames.length) {
            appState.currentFrameIndex = frameIdx;
            renderFrame(appState.frames[frameIdx]);
        }
    });

    // Playback buttons
    el.btnPlayPause.addEventListener("click", () => {
        if (appState.isPlaying) {
            pausePlayback();
        } else {
            startPlayback();
        }
    });

    el.btnStop.addEventListener("click", stopPlayback);

    el.speedSelect.addEventListener("change", (e) => {
        appState.playbackSpeed = parseFloat(e.target.value);
    });

    // Drag and Drop files
    el.dropZone.addEventListener("dragover", (e) => {
        e.preventDefault();
        el.dropZone.classList.add("dragover");
    });

    el.dropZone.addEventListener("dragleave", () => {
        el.dropZone.classList.remove("dragover");
    });

    el.dropZone.addEventListener("drop", (e) => {
        e.preventDefault();
        el.dropZone.classList.remove("dragover");
        if (e.dataTransfer.files.length > 0) {
            processDroppedFiles(e.dataTransfer.files);
        }
    });
}

// SETUP LIVE ROS2 EVENT SOURCE (SSE Streaming)
function setupLiveSSE() {
    try {
        // SSE Listener
        appState.liveStreamSource = new EventSource("/stream");

        appState.liveStreamSource.onopen = () => {
            console.log("Live stream connection opened successfully.");
            appState.mode = "LIVE";
            updateStatusUI();
        };

        appState.liveStreamSource.onmessage = (event) => {
            if (appState.mode !== "LIVE") {
                appState.mode = "LIVE";
                updateStatusUI();
            }
            try {
                const frameData = JSON.parse(event.data);
                appState.latestLiveFrame = frameData;
                renderFrame(frameData);
            } catch (e) {
                console.error("Error parsing live stream event:", e);
            }
        };

        appState.liveStreamSource.onerror = (e) => {
            // Silently retry in background. Revert to Offline mode for UI.
            if (appState.mode === "LIVE") {
                console.log("Live stream server disconnected. Waiting for connection...");
                appState.mode = "OFFLINE";
                updateStatusUI();
            }
        };
    } catch (err) {
        console.error("SSE connection setup failed:", err);
    }
}

function updateStatusUI() {
    if (appState.mode === "LIVE") {
        el.systemStatus.className = "status-badge status-live";
        el.systemStatus.textContent = "LIVE STREAMING";
        el.loadedStatus.innerHTML = `<span class="badge badge-success">Live Mode</span> 실시간 ADAS 노드에 연결되었습니다. <code>main.py</code> 파이프라인 데이터가 렌더링 중입니다.`;

        // Disable replay controls during live stream
        el.btnPlayPause.disabled = true;
        el.btnStop.disabled = true;
        el.timelineSlider.disabled = true;
        el.speedSelect.disabled = true;
    } else if (appState.mode === "REPLAY") {
        el.systemStatus.className = "status-badge status-replay";
        el.systemStatus.textContent = "REPLAY PLAYING";
        el.loadedStatus.innerHTML = `<span class="badge badge-success">Replay Mode</span> ${appState.frames.length} 프레임 데이터 로딩 완료. 시뮬레이션 제어 가능.`;

        el.btnPlayPause.disabled = false;
        el.btnStop.disabled = false;
        el.timelineSlider.disabled = false;
        el.speedSelect.disabled = false;
    } else {
        el.systemStatus.className = "status-badge status-offline";
        el.systemStatus.textContent = "OFFLINE";
        el.loadedStatus.innerHTML = `<span class="badge badge-error">Offline Mode</span> 대기 중: 로컬 <code>web_server.py</code>를 실행하거나 artifacts 로그를 업로드하세요.`;

        el.btnPlayPause.disabled = true;
        el.btnStop.disabled = true;
        el.timelineSlider.disabled = true;
        el.speedSelect.disabled = true;
    }
}

// Load files from browser dialogue
function handleFileSelection(e) {
    if (e.target.files.length > 0) {
        processDroppedFiles(e.target.files);
    }
}

// Processing dragging and uploading of JSON logs
function processDroppedFiles(fileList) {
    let files = Array.from(fileList);
    let trackingFile = files.find(f => f.name.includes("tracking_results") && f.name.endsWith(".json"));
    let predictionsFile = files.find(f => f.name.includes("predicted_trajectories") && f.name.endsWith(".json"));
    let warningsFile = files.find(f => f.name.includes("ttc_warnings") && f.name.endsWith(".json"));
    let latencyFile = files.find(f => f.name.includes("latency") && f.name.endsWith(".csv"));

    // Alert user if standard files are not found
    if (!trackingFile) {
        // Fallback: try to see if they uploaded any json to play
        trackingFile = files.find(f => f.name.endsWith(".json"));
        if (!trackingFile) {
            alert("보행자 추적 파일(tracking_results.json)이 포함되어야 시연 재생이 가능합니다.");
            return;
        }
    }

    const readers = [];
    let fileContents = {
        tracking: null,
        predictions: null,
        warnings: null,
        latency: null
    };

    const readFile = (file, key, isCSV = false) => {
        return new Promise((resolve) => {
            const reader = new FileReader();
            reader.onload = (e) => {
                try {
                    fileContents[key] = isCSV ? parseCSV(e.target.result) : JSON.parse(e.target.result);
                } catch (err) {
                    console.error(`Error parsing file ${file.name}:`, err);
                }
                resolve();
            };
            reader.readAsText(file);
        });
    };

    const promises = [];
    promises.push(readFile(trackingFile, "tracking"));
    if (predictionsFile) promises.push(readFile(predictionsFile, "predictions"));
    if (warningsFile) promises.push(readFile(warningsFile, "warnings"));
    if (latencyFile) promises.push(readFile(latencyFile, "latency", true));

    Promise.all(promises).then(() => {
        combineDataToFrames(fileContents);
    });
}

// Parse quick offline Latency CSV
function parseCSV(text) {
    const lines = text.split("\n");
    if (lines.length < 2) return [];
    const headers = lines[0].split(",").map(h => h.trim().replace(/^"|"$/g, ''));
    const results = [];

    for (let i = 1; i < lines.length; i++) {
        if (!lines[i].trim()) continue;
        const cols = lines[i].split(",").map(c => c.trim().replace(/^"|"$/g, ''));
        const obj = {};
        headers.forEach((h, idx) => {
            obj[h] = isNaN(cols[idx]) ? cols[idx] : parseFloat(cols[idx]);
        });
        results.push(obj);
    }
    return results;
}

// Combine split files into a clean timed frame index
function combineDataToFrames(data) {
    if (!data.tracking) return;

    // Step 1: Initialize frames from tracking results
    const rawFrames = data.tracking;
    let frames = [];

    // Helper to map predictions and warnings
    const predictionsByFrame = {};
    if (data.predictions) {
        data.predictions.forEach(p => {
            if (!predictionsByFrame[p.frame]) {
                predictionsByFrame[p.frame] = [];
            }
            predictionsByFrame[p.frame].push(p);
        });
    }

    const warningsByFrame = {};
    if (data.warnings) {
        data.warnings.forEach(w => {
            if (!warningsByFrame[w.frame_id]) {
                warningsByFrame[w.frame_id] = [];
            }
            warningsByFrame[w.frame_id].push(w);
        });
    }

    const latencyByFrame = {};
    if (data.latency) {
        data.latency.forEach(l => {
            latencyByFrame[l.frame] = l;
        });
    }

    rawFrames.forEach((rf, idx) => {
        const fid = rf.frame_id;
        const timeSec = rf.timestamp_sec;

        // Match lists
        const tracks = rf.tracks || [];
        const framePredictions = predictionsByFrame[fid] || [];
        const frameWarnings = warningsByFrame[fid] || [];
        const frameLatency = latencyByFrame[fid] || {};

        // Assemble final unified frame packet
        frames.push({
            frame_id: fid,
            timestamp_sec: timeSec,
            ego_speed_mps: appState.egoSpeed, // constant default or loaded
            tracks: tracks,
            trajectories: transformPredictions(framePredictions),
            warnings: frameWarnings,
            latency: frameLatency
        });
    });

    // Sort by timestamp
    frames.sort((a, b) => a.timestamp_sec - b.timestamp_sec);

    appState.frames = frames;
    appState.currentFrameIndex = 0;
    appState.mode = "REPLAY";

    // UI Slider configuration
    el.timelineSlider.max = frames.length - 1;
    el.timelineSlider.value = 0;

    // Update timeline timer displays
    const duration = frames[frames.length - 1].timestamp_sec - frames[0].timestamp_sec;
    el.timeTotal.textContent = formatDuration(duration);
    el.timeCurrent.textContent = "00:00";

    updateStatusUI();
    renderFrame(frames[0]);
}

// Convert serialized CSV flat row prediction array to track-wise arrays
function transformPredictions(flatPreds) {
    const list = {};
    flatPreds.forEach(p => {
        const tid = p.track_id;
        if (!list[tid]) {
            list[tid] = { track_id: tid, points: [] };
        }
        list[tid].points.push({
            x: p.predicted_x ?? p.x,
            y: p.predicted_y ?? p.y,
            t_sec: p.predicted_t_sec ?? p.t_sec
        });
    });
    return Object.values(list);
}

// Render dynamic HUD Frame to Dashboard panels
function renderFrame(frame) {
    if (!frame) return;

    // 1. Header Info
    el.hdrFrameId.textContent = frame.frame_id;
    el.hdrTimestamp.textContent = frame.timestamp_sec.toFixed(2) + "s";

    // 2. Ego Diagnostics
    const speedValue = Number(frame.ego_speed_mps);
    const speed = Number.isFinite(speedValue) ? speedValue : appState.egoSpeed;
    appState.egoSpeed = speed;
    const safetyRadiusValue = Number(frame.safety_radius_m);
    appState.safetyRadius = Number.isFinite(safetyRadiusValue) ? safetyRadiusValue : appState.safetyRadius;
    el.instSpeed.innerHTML = `${speed.toFixed(1)} <span class="unit">m/s</span>`;
    el.instSpeedKmh.textContent = `${(speed * 3.6).toFixed(1)} km/h`;
    el.paramEgoSpeed.textContent = speed.toFixed(1);
    el.paramSafetyRadius.textContent = `${appState.safetyRadius.toFixed(1)} m`;

    const acceleratorValue = Number(frame.ego_accelerator_pedal);
    const acceleratorPedal = Number.isFinite(acceleratorValue) ? Math.max(0, Math.min(1, acceleratorValue)) : 0.0;
    const acceleratorPressed = Boolean(frame.ego_accelerator_pressed);
    const brakePressed = Boolean(frame.ego_brake_pressed || frame.ego_brake_lights);
    appState.acceleratorPressed = acceleratorPressed;
    appState.acceleratorPedal = acceleratorPedal;
    appState.brakePressed = brakePressed;
    appState.brakeLights = Boolean(frame.ego_brake_lights);
    el.pedalAccel.className = `pedal-card${acceleratorPressed ? " pedal-active-accel" : ""}`;
    el.pedalBrake.className = `pedal-card${brakePressed ? " pedal-active-brake" : ""}`;
    el.instAccelState.textContent = acceleratorPressed ? "ON" : "OFF";
    el.instAccelValue.textContent = `${(acceleratorPedal * 100).toFixed(0)}%`;
    el.instBrakeState.textContent = brakePressed ? "ON" : "OFF";
    el.instBrakeValue.textContent = frame.ego_brake_lights ? "lights on" : "lights off";

    // Find active warnings
    const warnings = frame.warnings || [];
    const maxWarning = warnings.length > 0 ? warnings.reduce((prev, curr) => (prev.level > curr.level) ? prev : curr) : null;

    let warningLevel = 0;
    let warningLabel = "SYSTEM SAFE";
    let warningAction = "보행자 감지 없음 (정상 주행)";
    let warningColor = "safe";
    let targetDecel = 0.0;

    if (maxWarning) {
        warningLevel = maxWarning.level;
        targetDecel = maxWarning.target_accel_mps2 || 0.0;

        // 가장 위험한 보행자의 TTC/거리 정보
        const ttcStr = (maxWarning.min_ttc_sec !== undefined && isFinite(maxWarning.min_ttc_sec) && maxWarning.min_ttc_sec < 90)
            ? `TTC ${maxWarning.min_ttc_sec.toFixed(1)}s`
            : "";
        const trackIdStr = `ID ${maxWarning.track_id}`;

        // TTC 3단계 한국어 경고 문구 적용
        if (warningLevel === 1) {
            warningLabel = "보행자 확인";
            warningAction = `${trackIdStr} 감지 ${ttcStr}`;
            warningColor = "level1";
        } else if (warningLevel === 2) {
            warningLabel = "보행자 경고";
            warningAction = `${trackIdStr} 접근 중 ${ttcStr}`;
            warningColor = "level2";
        } else if (warningLevel === 3) {
            warningLabel = "보행자 위험";
            warningAction = `${trackIdStr} 충돌 위험! ${ttcStr}`;
            warningColor = "level3";
        } else {
            warningLabel = maxWarning.label;
            warningAction = translateAction(maxWarning.action);
            warningColor = maxWarning.color || "safe";
        }

        // Sound beep trigger
        if (warningLevel > 0) {
            playWarningBeep(warningLevel);
        }
    }

    // UI Warning Panels update
    el.instDecel.innerHTML = `${targetDecel.toFixed(2)} <span class="unit">m/s²</span>`;
    // Deceleration percentage mapping (0 to -8.0 max decel in config)
    const decelPercent = Math.min(Math.abs(targetDecel) / 8.0 * 100, 100);
    el.decelBar.style.width = `${decelPercent}%`;

    // Warning banner styling
    el.warningBanner.className = `warning-banner banner-${warningColor}`;
    el.warnTitle.textContent = warningLabel.replace(/_/g, " ");
    el.warnAction.textContent = warningAction;

    // Emergency glow styling
    el.emergencyGlow.className = `glow-${warningColor}`;

    // 3. Telemetry Matrix Update
    updateTelemetryList(frame.tracks || [], warnings);

    // 4. Latency breakdown
    updateLatencyUI(frame.latency || {});

    // 5. Draw radar BEV with pedestrian markers
    drawRadar(frame);
}

// Convert action strings to beautiful Korean HUD display descriptions
function translateAction(action) {
    switch (action) {
        case "normal": return "정상 운행 구역";
        case "warning_candidate": return "보행자 주의 (감속 준비)";
        case "s_curve_decel_candidate": return "보행자 접근 - 부드러운 속도 감속 시작!";
        case "max_decel_candidate": return "긴급 제동 작동! (AEB 충돌 위험!)";
        default: return action;
    }
}

function updateTelemetryList(tracks, warnings) {
    if (tracks.length === 0) {
        el.trackList.innerHTML = `<div class="no-tracks">인식된 보행자 없음</div>`;
        return;
    }

    const warningByTrack = {};
    warnings.forEach(w => {
        warningByTrack[w.track_id] = w;
    });

    let html = "";
    tracks.forEach(track => {
        const tid = track.track_id;
        const dist = Math.hypot(track.x, track.y).toFixed(1);
        const wInfo = warningByTrack[tid];

        let ttcVal = "∞";
        let level = 0;
        let threatLabel = "SAFE";
        let threatClass = "safe";

        if (wInfo) {
            level = wInfo.level;
            ttcVal = isFinite(wInfo.min_ttc_sec) ? `${wInfo.min_ttc_sec.toFixed(2)}s` : "∞";
            threatClass = `level${level}`;
            // TTC 단계별 한국어 라벨
            if (level === 1) threatLabel = "보행자 확인";
            else if (level === 2) threatLabel = "보행자 경고";
            else if (level === 3) threatLabel = "보행자 위험";
            else threatLabel = wInfo.label.replace("LEVEL", "L");
        }

        html += `
            <div class="track-row level-${level}">
                <div class="track-id">#${tid}</div>
                <div class="track-dist">${dist}m</div>
                <div class="track-ttc">${ttcVal}</div>
                <div class="track-threat ${threatClass}">${threatLabel}</div>
            </div>
        `;
    });
    el.trackList.innerHTML = html;
}

function updateLatencyUI(lat) {
    const p = lat.perception_ms || 0;
    const pr = lat.prediction_ms || 0;
    const tr = lat.tracking_ms || 0;
    const total = lat.total_callback_ms || (p + pr + tr);

    // Scale widths
    const maxVal = Math.max(total, 50.0); // max 50ms relative scale

    el.latPerception.style.width = `${(p / maxVal) * 100}%`;
    el.latValPerception.textContent = `${p.toFixed(1)}ms`;

    el.latPrediction.style.width = `${(pr / maxVal) * 100}%`;
    el.latValPrediction.textContent = `${pr.toFixed(1)}ms`;

    el.latTracking.style.width = `${(tr / maxVal) * 100}%`;
    el.latValTracking.textContent = `${tr.toFixed(1)}ms`;

    el.latTotal.textContent = `${total.toFixed(1)}ms`;
}

// DRAW THE BIRD'S EYE VIEW (BEV) INFOTAINMENT NAVIGATION SCREEN
function drawRadar(frame) {
    // Clear canvas
    ctx.clearRect(0, 0, el.radarCanvas.width, el.radarCanvas.height);

    const w = el.radarCanvas.width;
    const h = el.radarCanvas.height;

    // 1. Draw Stylized Modern Navigation Road lines scrolling down
    drawNavigationHighway(w, h);

    // 2. Draw Safety Zone Radius Area (glowing bounds)
    drawSafetyZones();

    // 3. Draw ego vehicle at bottom center
    drawEgoCar();

    // 4. Draw static obstacle cloud candidate before pedestrians
    drawStaticObstacleCluster(frame.static_obstacle);

    // 5. Draw Pedestrians, past tracks, and predicted paths
    const tracks = frame.tracks || [];
    const warnings = frame.warnings || [];
    const trajectories = frame.trajectories || [];

    const warningByTrack = {};
    warnings.forEach(w => {
        warningByTrack[w.track_id] = w;
    });

    const trajectoriesByTrack = {};
    trajectories.forEach(t => {
        trajectoriesByTrack[t.track_id] = t;
    });

    // (예측선 비활성화 - 점(puck)만 표시)

    // Draw Pedestrian Pucks and Blinking Warnings
    tracks.forEach(track => {
        const tid = track.track_id;
        const tWarn = warningByTrack[tid];
        const pCanvas = toCanvasCoords(track.x, track.y);

        let level = 0;
        let pColor = "#a0aec0"; // 미감지 보행자: 회색
        let pulseSpeed = 0;
        let ttcLabel = ""; // TTC 단계별 한국어 문구

        if (tWarn) {
            level = tWarn.level;
            if (level === 1) { pColor = "#22c55e"; pulseSpeed = 1000; ttcLabel = "보행자 확인"; }   // 초록 점등
            if (level === 2) { pColor = "#f97316"; pulseSpeed = 500;  ttcLabel = "보행자 경고"; }   // 주황 점등
            if (level === 3) { pColor = "#ef4444"; pulseSpeed = 250;  ttcLabel = "보행자 위험"; }   // 빨강 점등
        }

        // Blink logic based on elapsed animation time
        const time = performance.now();
        const blinkActive = pulseSpeed === 0 || Math.floor(time / pulseSpeed) % 2 === 0;

        // Hazard outer glowing ripple circle for level 2 and 3
        if (level >= 2) {
            const rippleRadius = 20 + 15 * Math.abs(Math.sin(time / 150));
            ctx.beginPath();
            ctx.arc(pCanvas.x, pCanvas.y, rippleRadius, 0, 2 * Math.PI);
            ctx.strokeStyle = level === 3 ? "rgba(239, 68, 68, 0.4)" : "rgba(249, 115, 22, 0.4)";
            ctx.lineWidth = 1.5;
            ctx.stroke();
        }

        // Level 1 also gets a subtle outer ring (초록 링)
        if (level === 1) {
            const ringRadius = 18 + 6 * Math.abs(Math.sin(time / 300));
            ctx.beginPath();
            ctx.arc(pCanvas.x, pCanvas.y, ringRadius, 0, 2 * Math.PI);
            ctx.strokeStyle = "rgba(34, 197, 94, 0.35)";
            ctx.lineWidth = 1.5;
            ctx.stroke();
        }

        // Pedestrian Base puck shape
        ctx.beginPath();
        ctx.arc(pCanvas.x, pCanvas.y, 10, 0, 2 * Math.PI);
        // Fill or flash
        ctx.fillStyle = blinkActive ? pColor : "rgba(255,255,255,0.1)";
        ctx.shadowColor = pColor;
        ctx.shadowBlur = level >= 1 ? 16 : 8;
        ctx.fill();

        // Glow border
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.shadowBlur = 0; // reset

        // TTC 단계별 한국어 문구 + TTC 수치 표시
        if (level >= 1 && ttcLabel) {
            ctx.font = "bold 12px Outfit, 'Noto Sans KR', sans-serif";
            const ttcText = tWarn.min_ttc_sec !== undefined && isFinite(tWarn.min_ttc_sec)
                ? `${ttcLabel} (TTC:${tWarn.min_ttc_sec.toFixed(1)}s)`
                : ttcLabel;
            const textWidth = ctx.measureText(ttcText).width;

            // 배경 박스
            const boxX = pCanvas.x - textWidth / 2 - 8;
            const boxY = pCanvas.y - 34;
            const boxW = textWidth + 16;
            const boxH = 20;

            ctx.fillStyle = "rgba(0, 0, 0, 0.85)";
            // 둥근 모서리 배경
            ctx.beginPath();
            ctx.roundRect(boxX, boxY, boxW, boxH, 4);
            ctx.fill();

            ctx.strokeStyle = pColor;
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.roundRect(boxX, boxY, boxW, boxH, 4);
            ctx.stroke();

            // 문구 텍스트
            ctx.fillStyle = pColor;
            ctx.fillText(ttcText, pCanvas.x - textWidth / 2, pCanvas.y - 19);
        } else {
            // 미감지 보행자 - 단순 ID 표시
            ctx.fillStyle = "#ffffff";
            ctx.font = "9px Share Tech Mono";
            ctx.fillText(`ID ${tid}`, pCanvas.x - 10, pCanvas.y - 14);
        }
    });
}

function drawStaticObstacleCluster(obstacle) {
    if (!obstacle || !obstacle.point_count) return;

    const color = staticObstacleColor(obstacle);
    const topLeft = toCanvasCoords(obstacle.x_max, obstacle.y_max);
    const bottomRight = toCanvasCoords(obstacle.x_min, obstacle.y_min);
    const minX = Math.min(topLeft.x, bottomRight.x);
    const maxX = Math.max(topLeft.x, bottomRight.x);
    const minY = Math.min(topLeft.y, bottomRight.y);
    const maxY = Math.max(topLeft.y, bottomRight.y);
    const width = Math.max(maxX - minX, 8);
    const height = Math.max(maxY - minY, 8);
    const center = toCanvasCoords(obstacle.centroid_x, obstacle.centroid_y);
    const isSuppressed = Boolean(obstacle.suppressed_low_speed);

    ctx.save();
    ctx.setLineDash(isSuppressed ? [6, 5] : []);
    ctx.fillStyle = color.fill;
    ctx.strokeStyle = color.stroke;
    ctx.lineWidth = obstacle.level > 0 ? 2.5 : 1.5;
    ctx.shadowColor = color.stroke;
    ctx.shadowBlur = obstacle.level > 0 ? 18 : 8;
    ctx.fillRect(minX, minY, width, height);
    ctx.strokeRect(minX, minY, width, height);
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.arc(center.x, center.y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = color.stroke;
    ctx.fill();
    ctx.shadowBlur = 0;

    const ttcText = isSuppressed ? "SUPP" : (Number.isFinite(obstacle.ttc_sec) ? `TTC ${obstacle.ttc_sec.toFixed(1)}s` : "TTC inf");
    const label = `STATIC ${obstacle.point_count}pts ${obstacle.distance_m.toFixed(1)}m ${ttcText}`;
    ctx.font = "bold 11px Outfit, 'Noto Sans KR', sans-serif";
    const textWidth = ctx.measureText(label).width;
    const labelX = Math.max(8, Math.min(center.x - textWidth / 2 - 6, el.radarCanvas.width - textWidth - 16));
    const labelY = Math.max(8, center.y - height / 2 - 28);
    ctx.fillStyle = "rgba(0, 0, 0, 0.82)";
    ctx.beginPath();
    ctx.roundRect(labelX, labelY, textWidth + 12, 20, 4);
    ctx.fill();
    ctx.strokeStyle = color.stroke;
    ctx.stroke();
    ctx.fillStyle = color.stroke;
    ctx.fillText(label, labelX + 6, labelY + 14);
    ctx.restore();
}

function staticObstacleColor(obstacle) {
    if (obstacle.level === 3) return { stroke: "#ef4444", fill: "rgba(239, 68, 68, 0.22)" };
    if (obstacle.level === 2) return { stroke: "#f97316", fill: "rgba(249, 115, 22, 0.20)" };
    if (obstacle.level === 1) return { stroke: "#fbbf24", fill: "rgba(251, 191, 36, 0.18)" };
    if (obstacle.suppressed_low_speed) return { stroke: "#60a5fa", fill: "rgba(96, 165, 250, 0.14)" };
    return { stroke: "#00f2fe", fill: "rgba(0, 242, 254, 0.14)" };
}

// Draw a beautiful perspective digital roadway map that scrolls dynamically
function drawNavigationHighway(w, h) {
    // Radar grid lines
    ctx.strokeStyle = "rgba(255, 255, 255, 0.02)";
    ctx.lineWidth = 1;
    for (let i = 0; i < w; i += 40) {
        ctx.beginPath();
        ctx.moveTo(i, 0);
        ctx.lineTo(i, h);
        ctx.stroke();
    }
    for (let j = 0; j < h; j += 40) {
        ctx.beginPath();
        ctx.moveTo(0, j);
        ctx.lineTo(w, j);
        ctx.stroke();
    }

    // Grid concentric range rings representing distances (radar circles)
    ctx.strokeStyle = "rgba(0, 242, 254, 0.05)";
    for (let r = 5; r <= 50; r += 10) {
        ctx.beginPath();
        ctx.arc(egoCarPos.x, egoCarPos.y, r * RADAR_SCALE, 0, 2 * Math.PI);
        ctx.stroke();
    }

    // PERSPECTIVE ROADWAY drawing (representing modern vehicle lane assistant HUD)
    const horizonY = 50; // road starts narrow here
    const roadTopWidth = 30;
    const roadBottomWidth = 240;

    // Draw asphalt surface
    ctx.beginPath();
    ctx.moveTo(egoCarPos.x - roadTopWidth/2, horizonY);
    ctx.lineTo(egoCarPos.x + roadTopWidth/2, horizonY);
    ctx.lineTo(egoCarPos.x + roadBottomWidth/2, h);
    ctx.lineTo(egoCarPos.x - roadBottomWidth/2, h);
    ctx.closePath();
    ctx.fillStyle = "rgba(20, 22, 32, 0.25)";
    ctx.fill();

    // Road Outer Borders (Cyan Glowing Lane markers)
    ctx.beginPath();
    ctx.moveTo(egoCarPos.x - roadTopWidth/2, horizonY);
    ctx.lineTo(egoCarPos.x - roadBottomWidth/2, h);
    ctx.strokeStyle = "rgba(0, 242, 254, 0.15)";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(egoCarPos.x + roadTopWidth/2, horizonY);
    ctx.lineTo(egoCarPos.x + roadBottomWidth/2, h);
    ctx.stroke();

    // Scrolling center lane dividers (Yellow Dashed Lines)
    const time = performance.now();
    // Scroll speed depends on vehicle's current speed
    const velocityMultiplier = Math.max(appState.egoSpeed, 1.0);
    appState.roadOffset = (appState.roadOffset + (velocityMultiplier * 0.8)) % 80;

    ctx.strokeStyle = "rgba(251, 191, 36, 0.25)";
    ctx.lineWidth = 2;

    for (let offset = -appState.roadOffset; offset < h - horizonY; offset += 40) {
        const laneY1 = horizonY + offset;
        const laneY2 = laneY1 + 18;
        if (laneY1 < horizonY) continue;
        if (laneY2 > h) break;

        // Interpolate perspective width ratio
        const ratio1 = (laneY1 - horizonY) / (h - horizonY);
        const ratio2 = (laneY2 - horizonY) / (h - horizonY);

        const w1 = roadTopWidth + ratio1 * (roadBottomWidth - roadTopWidth);
        const w2 = roadTopWidth + ratio2 * (roadBottomWidth - roadTopWidth);

        // Draw Left Lane Dashed line
        ctx.beginPath();
        ctx.moveTo(egoCarPos.x - w1/6, laneY1);
        ctx.lineTo(egoCarPos.x - w2/6, laneY2);
        ctx.stroke();

        // Draw Right Lane Dashed line
        ctx.beginPath();
        ctx.moveTo(egoCarPos.x + w1/6, laneY1);
        ctx.lineTo(egoCarPos.x + w2/6, laneY2);
        ctx.stroke();
    }
}

// Draw Ego Vehicle Symbol at the bottom
function drawEgoCar() {
    const cx = egoCarPos.x;
    const cy = egoCarPos.y;

    // Headlight cones extending forward (stylized FOV)
    const gradient = ctx.createRadialGradient(cx, cy, 5, cx, cy - 80, 100);
    gradient.addColorStop(0, "rgba(0, 242, 254, 0.12)");
    gradient.addColorStop(1, "rgba(0, 242, 254, 0)");

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx - 50, cy - 100);
    ctx.lineTo(cx + 50, cy - 100);
    ctx.closePath();
    ctx.fillStyle = gradient;
    ctx.fill();

    // Glowing Ego vehicle outline
    ctx.shadowColor = "#00f2fe";
    ctx.shadowBlur = 10;

    // Car Body Draw
    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#00f2fe";
    ctx.lineWidth = 2;

    ctx.beginPath();
    // Front bumper
    ctx.moveTo(cx - 10, cy - 25);
    ctx.lineTo(cx + 10, cy - 25);
    // Right side
    ctx.lineTo(cx + 14, cy - 15);
    ctx.lineTo(cx + 14, cy + 18);
    // Rear bumper
    ctx.lineTo(cx + 10, cy + 22);
    ctx.lineTo(cx - 10, cy + 22);
    // Left side
    ctx.lineTo(cx - 14, cy + 18);
    ctx.lineTo(cx - 14, cy - 15);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Reset shadow
    ctx.shadowBlur = 0;

    // Windshield / Glass
    ctx.fillStyle = "rgba(0, 242, 254, 0.4)";
    ctx.beginPath();
    ctx.moveTo(cx - 8, cy - 12);
    ctx.lineTo(cx + 8, cy - 12);
    ctx.lineTo(cx + 6, cy - 3);
    ctx.lineTo(cx - 6, cy - 3);
    ctx.closePath();
    ctx.fill();
}

// Draw Safe braking boundary lines on road
function drawSafetyZones() {
    const cy = egoCarPos.y;
    // Map safety radius (meters to pixels)
    const sRadPx = appState.safetyRadius * RADAR_SCALE;

    ctx.beginPath();
    // Safety bounding lateral lines extending forward
    ctx.moveTo(egoCarPos.x - sRadPx, cy);
    ctx.lineTo(egoCarPos.x - sRadPx, cy - 300);
    ctx.moveTo(egoCarPos.x + sRadPx, cy);
    ctx.lineTo(egoCarPos.x + sRadPx, cy - 300);

    ctx.strokeStyle = "rgba(16, 185, 129, 0.12)";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 6]);
    ctx.stroke();
    ctx.setLineDash([]);
}

// Convert spatial relative points to localized canvas coordinates
function toCanvasCoords(relX, relY) {
    // relX: forward distance (+ represents forward from car in meters)
    // relY: lateral distance (+ represents left of car in meters, - represents right)

    // Map forward distance to visual Y (going up the canvas from Ego vehicle)
    const cy = egoCarPos.y - (relX * RADAR_SCALE);

    // Map lateral distance to visual X (left is -Y in visual coordinates, right is +Y. Wait:
    // LiDAR Y positive is LEFT, so positive Y subtracts from EgoCarX)
    const cx = egoCarPos.x - (relY * RADAR_SCALE);

    return { x: cx, y: cy };
}

// Duration string formatting
function formatDuration(sec) {
    if (isNaN(sec) || !isFinite(sec)) return "00:00";
    const mins = Math.floor(sec / 60);
    const secs = Math.floor(sec % 60);
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}

// Playback Render Animation Loop (runs on requestAnimationFrame)
function animationLoop(timestamp) {
    if (appState.mode === "REPLAY" && appState.isPlaying) {
        const dt = timestamp - appState.lastAnimTime;
        appState.lastAnimTime = timestamp;

        const tickStep = (dt / 100) * appState.playbackSpeed;

        let newIdx = appState.currentFrameIndex + tickStep;
        if (newIdx >= appState.frames.length) {
            newIdx = 0;
            appState.isPlaying = false;
            el.btnPlayPause.textContent = "▶ PLAY";
        }

        appState.currentFrameIndex = Math.floor(newIdx);
        el.timelineSlider.value = appState.currentFrameIndex;

        const currentFrame = appState.frames[appState.currentFrameIndex];
        const elapsed = currentFrame.timestamp_sec - appState.frames[0].timestamp_sec;
        el.timeCurrent.textContent = formatDuration(elapsed);

        renderFrame(currentFrame);
    } else {
        appState.lastAnimTime = timestamp;
        if (appState.mode === "LIVE") {
            if (appState.latestLiveFrame) drawRadar(appState.latestLiveFrame);
        } else if (appState.frames.length > 0) {
            drawRadar(appState.frames[appState.currentFrameIndex]);
        } else {
            drawRadar({ tracks: [], trajectories: [], warnings: [] });
        }
    }

    requestAnimationFrame(animationLoop);
}

function startPlayback() {
    if (appState.frames.length === 0) return;
    appState.isPlaying = true;
    appState.lastAnimTime = performance.now();
    el.btnPlayPause.textContent = "⏸ PAUSE";
}

function pausePlayback() {
    appState.isPlaying = false;
    el.btnPlayPause.textContent = "▶ PLAY";
}

function stopPlayback() {
    appState.isPlaying = false;
    appState.currentFrameIndex = 0;
    el.timelineSlider.value = 0;
    el.timeCurrent.textContent = "00:00";
    el.btnPlayPause.textContent = "▶ PLAY";
    if (appState.frames.length > 0) {
        renderFrame(appState.frames[0]);
    }
}
