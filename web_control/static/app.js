const $ = (id) => document.getElementById(id);

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const data = await response.json();
  if (!response.ok && !data.msg) data.msg = response.statusText;
  return data;
}

async function sendAction(action) {
  const result = await api("/api/action", {
    method: "POST",
    body: JSON.stringify({ action, source: "web" }),
  });
  $("chatReply").textContent = result.ok ? `已执行：${action}` : `执行失败：${result.msg || "busy"}`;
  await refreshStatus();
}

async function setMode(mode) {
  const result = await api("/api/mode", {
    method: "POST",
    body: JSON.stringify({ mode }),
  });
  $("chatReply").textContent = result.ok ? `模式：${mode}` : `模式切换失败：${result.msg || "busy"}`;
  await refreshStatus();
}

async function refreshStatus() {
  const data = await api("/api/status");
  const robot = data.robot;
  const vision = data.vision;
  $("robotMode").textContent = `mode: ${robot.current_mode}`;
  $("running").textContent = robot.running ? "yes" : "no";
  $("source").textContent = robot.last_source || "--";
  $("simulation").textContent = robot.simulation ? "yes" : "no";
  $("gesture").textContent = vision.gesture || "--";
  $("cameraStatus").textContent = `camera: ${vision.frame_ok ? "ok" : "check"}`;
  $("visionStatus").textContent = `mediapipe: ${vision.mediapipe_available ? "ok" : "optional"}`;
  if (vision.face) {
    $("faceOffset").textContent = `${vision.face.offset_x}, ${vision.face.offset_y}`;
  } else {
    $("faceOffset").textContent = "--";
  }
  $("joints").textContent = `[${robot.last_joints.map((v) => Math.round(v)).join(", ")}]`;
  $("log").innerHTML = robot.log
    .map((item) => `<li><strong>${item.time}</strong> ${item.event}: ${item.message}</li>`)
    .join("");
}

async function loadSequence() {
  const name = $("sequenceName").value.trim() || "demo";
  const result = await api(`/api/sequences/${encodeURIComponent(name)}`);
  $("sequenceView").textContent = result.ok ? JSON.stringify(result.steps, null, 2) : "[]";
}

async function recordStep() {
  const name = $("sequenceName").value.trim() || "demo";
  const result = await api(`/api/sequences/${encodeURIComponent(name)}/record`, {
    method: "POST",
    body: JSON.stringify({ duration: 800 }),
  });
  $("sequenceView").textContent = JSON.stringify(result.steps || [], null, 2);
  await refreshStatus();
}

async function playSequence() {
  const name = $("sequenceName").value.trim() || "demo";
  const result = await api(`/api/sequences/${encodeURIComponent(name)}/play`, {
    method: "POST",
    body: JSON.stringify({}),
  });
  $("chatReply").textContent = result.ok ? `回放动作组：${name}` : `回放失败：${result.msg}`;
  await refreshStatus();
}

document.querySelectorAll("[data-action]").forEach((button) => {
  button.addEventListener("click", () => sendAction(button.dataset.action));
});

document.querySelectorAll("[data-mode]").forEach((button) => {
  button.addEventListener("click", () => setMode(button.dataset.mode));
});

$("chatForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  const text = $("chatInput").value;
  const result = await api("/api/chat", {
    method: "POST",
    body: JSON.stringify({ text }),
  });
  $("chatReply").textContent = result.reply || result.msg || "完成";
  await refreshStatus();
});

$("recordStep").addEventListener("click", recordStep);
$("playSequence").addEventListener("click", playSequence);
$("loadSequence").addEventListener("click", loadSequence);

refreshStatus();
setInterval(refreshStatus, 1500);

