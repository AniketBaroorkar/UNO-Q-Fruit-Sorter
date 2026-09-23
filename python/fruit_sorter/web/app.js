const $ = (id) => document.getElementById(id);

let consecutiveFailures = 0;

function text(id, value) {
  const element = $(id);
  if (element) element.textContent = value;
}

function yesNo(value, yes = "Yes", no = "No", unknown = "Unknown") {
  if (value === null || value === undefined) return unknown;
  return value ? yes : no;
}

function duration(seconds) {
  const total = Math.max(0, Math.floor(seconds || 0));
  const h = String(Math.floor(total / 3600)).padStart(2, "0");
  const m = String(Math.floor((total % 3600) / 60)).padStart(2, "0");
  const s = String(total % 60).padStart(2, "0");
  return `${h}:${m}:${s}`;
}

function clock(iso) {
  if (!iso) return "—";
  return new Date(iso).toLocaleTimeString();
}

function percent(value) {
  return `${Math.round((Number(value) || 0) * 100)}%`;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function stateClass(state) {
  if (["idle", "acquiring", "homing"].includes(state)) return "good";
  if (["picking", "placing"].includes(state)) return "warning";
  if (["emergency_stop", "error"].includes(state)) return "danger";
  return "neutral";
}

function detectionDecision(row) {
  const status = row.status || "unknown";
  if (status === "ready") return ["READY", "ready"];
  if (["moving", "untracked"].includes(status)) return [status.toUpperCase(), "waiting"];
  return [status.replaceAll("_", " ").toUpperCase(), "rejected"];
}

function renderDetections(rows) {
  const body = $("detections-body");
  if (!rows || rows.length === 0) {
    body.innerHTML = '<tr><td colspan="8" class="empty-row">No fruit detected</td></tr>';
    return;
  }

  body.innerHTML = rows.map((row) => {
    const [decision, decisionClass] = detectionDecision(row);
    const robot = row.robot_x_mm === null || row.robot_x_mm === undefined
      ? "—"
      : `${row.robot_x_mm.toFixed(1)}, ${row.robot_y_mm.toFixed(1)} mm`;
    return `
      <tr>
        <td><span class="fruit-label"><span class="fruit-dot ${row.fruit}"></span>${row.fruit}</span></td>
        <td>${row.track_id ?? "—"}</td>
        <td>${percent(row.confidence)}</td>
        <td>${percent(row.colour_ratio)}</td>
        <td>${yesNo(row.stable)}</td>
        <td>${row.center_x_px.toFixed(0)}, ${row.center_y_px.toFixed(0)}</td>
        <td>${robot}</td>
        <td><span class="decision ${decisionClass}">${decision}</span></td>
      </tr>`;
  }).join("");
}

function renderTarget(target) {
  const box = $("active-target");
  if (!target) {
    box.className = "empty-card";
    box.textContent = "No active target";
    return;
  }
  box.className = "target-card";
  box.innerHTML = `
    <strong>${target.fruit}</strong> · Track ${target.track_id ?? "—"}
    <div class="target-grid">
      <div><span>Robot X</span>${Number(target.x_mm).toFixed(1)} mm</div>
      <div><span>Robot Y</span>${Number(target.y_mm).toFixed(1)} mm</div>
      <div><span>Confidence</span>${percent(target.confidence)}</div>
      <div><span>Colour match</span>${percent(target.colour_ratio)}</div>
    </div>`;
}

function renderLastSort(lastSort) {
  const box = $("last-sort");
  if (!lastSort) {
    box.className = "empty-card";
    box.textContent = "No completed cycles";
    return;
  }
  box.className = "target-card";
  box.innerHTML = `
    <strong>${lastSort.fruit}</strong> · Track ${lastSort.track_id ?? "—"}
    <div class="target-grid">
      <div><span>Pick position</span>${lastSort.x_mm}, ${lastSort.y_mm} mm</div>
      <div><span>Cycle time</span>${lastSort.cycle_seconds} s</div>
      <div><span>Completed</span>${clock(lastSort.completed_at)}</div>
      <div><span>Destination</span>${lastSort.fruit} box</div>
    </div>`;
}

function renderServos(robot) {
  const names = robot.servo_names || [];
  const angles = robot.pose_deg || [];
  $("servo-bars").innerHTML = names.map((name, index) => {
    const angle = Number(angles[index] || 0);
    const width = Math.min(100, Math.max(0, angle / 180 * 100));
    return `
      <div class="servo-row">
        <span class="servo-name">${name.replaceAll("_", " ")}</span>
        <div class="servo-track"><div class="servo-fill" style="width:${width}%"></div></div>
        <span class="servo-angle">${angle.toFixed(1)}°</span>
      </div>`;
  }).join("");
}

function renderEvents(events) {
  const box = $("events");
  if (!events || events.length === 0) {
    box.innerHTML = '<div class="empty-card">No events yet</div>';
    return;
  }
  box.innerHTML = events.map((event) => `
    <div class="event ${event.level}">
      <time>${clock(event.time)}</time>
      <p>${escapeHtml(event.message)}</p>
    </div>`).join("");
}

function updateDashboard(data) {
  consecutiveFailures = 0;
  const system = data.system;
  const safety = data.safety;
  const counts = data.sort_counts;

  $("connection-dot").className = "status-dot online";
  text("connection-text", "Dashboard connected");
  text("last-update", `Updated ${clock(system.last_update)}`);

  text("count-apple", counts.apple);
  text("count-banana", counts.banana);
  text("count-orange", counts.orange);
  text("count-total", counts.total);

  text("fps-badge", `${system.processing_fps.toFixed(1)} FPS`);
  text("frame-number", system.frame_number);
  text("inference-ms", `${system.inference_ms.toFixed(1)} ms`);
  text("detection-count", data.detections.length);
  text("uptime", duration(system.uptime_seconds));

  text("robot-state", system.state.replaceAll("_", " "));
  const stateBadge = $("state-badge");
  stateBadge.textContent = system.state.toUpperCase().replaceAll("_", " ");
  stateBadge.className = `badge ${stateClass(system.state)}`;

  text("camera-status", system.camera_connected ? "Online" : "Offline");
  text("robot-link", system.robot_connected ? "Online" : "Offline");
  text("outputs-status", safety.outputs_enabled ? "Enabled" : "Disabled");
  text("motion-status", safety.motion_busy ? "Moving" : "Idle");
  text("calibration-status", system.calibration_complete ? "Complete" : "Motion locked");
  text("run-mode", system.dry_run ? "Dry run" : "Hardware");

  const unsafe = safety.emergency_stop || safety.physical_estop || safety.fault !== "none";
  const safetyPanel = $("safety-panel");
  safetyPanel.className = `panel safety-panel ${unsafe ? "unsafe" : "safe"}`;
  text("safety-heading", unsafe ? "Stop / fault active" : "Safety normal");
  const safetyBadge = $("safety-badge");
  safetyBadge.textContent = unsafe ? "ATTENTION" : "NORMAL";
  safetyBadge.className = `badge ${unsafe ? "danger" : ""}`;
  text("fault-code", String(safety.fault ?? "unknown").toUpperCase());
  text("physical-estop", yesNo(safety.physical_estop, "Pressed / open", "Normal"));
  text("last-error", safety.last_error || "None");

  $("remote-control").classList.toggle("hidden", !safety.remote_estop_enabled);
  $("video-offline").classList.toggle("hidden", system.camera_connected);

  renderDetections(data.detections);
  renderTarget(data.active_target);
  renderLastSort(data.last_sort);
  renderServos(data.robot);
  renderEvents(data.events);
}

async function poll() {
  try {
    const response = await fetch("/api/status", {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    updateDashboard(await response.json());
  } catch (error) {
    consecutiveFailures += 1;
    $("connection-dot").className = "status-dot offline";
    text("connection-text", "Dashboard disconnected");
    text("last-update", consecutiveFailures > 1 ? "Retrying…" : String(error));
  }
}

$("estop-button").addEventListener("click", async () => {
  if (!confirm("Send an emergency-stop command to the robot?")) return;
  const token = $("control-token").value;
  sessionStorage.setItem("fruit-sorter-token", token);
  const response = await fetch("/api/emergency-stop", {
    method: "POST",
    headers: {"X-Control-Token": token},
  });
  const result = await response.json();
  if (!response.ok) {
    alert(result.detail || "Emergency-stop request failed");
  }
});

$("control-token").value = sessionStorage.getItem("fruit-sorter-token") || "";
$("live-video").addEventListener("load", () => $("video-offline").classList.add("hidden"));
$("live-video").addEventListener("error", () => $("video-offline").classList.remove("hidden"));

poll();
setInterval(poll, 500);
