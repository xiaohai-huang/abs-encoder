/**
 * @file gui_main.cpp
 * @brief Localhost web GUI for the PC simulation: a tiny HTTP server that
 *        drives the modelled shaft through the firmware sample loop and
 *        serves a browser page with the gear train, controls, decoded
 *        position and the I2C register map.
 *
 * Build and run:  cd sim && make gui
 * Then open:      http://127.0.0.1:8080/
 */
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET SocketHandle;
#define CLOSE_SOCKET closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int SocketHandle;
#define CLOSE_SOCKET close
#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif
#endif

#include "app_entry.h"
#include "gear_config.h"
#include "gear_train_sim.h"
#include "i2c_pos.h"
#include "mt6701.h"
#include "mt6701_slave_sim.h"

/* Defined at the end of the file; declared here so the request handler can
 * serve it. */
extern const char* const _page;

namespace
{
const EncoderRole Roles[] = {EncoderRole::Sun, EncoderRole::Gear1,
                             EncoderRole::Gear2, EncoderRole::Gear3};
const char* const RoleKeys[] = {"sun", "g1", "g2", "g3"};

struct FaultState
{
    bool    Track;
    bool    Crc;
    bool    Stuck;
    uint8_t Field;
};

FaultState _faults[PositionRegister::WheelCount] = {};

uint16_t Le16(const uint8_t* bytes, unsigned index)
{
    return static_cast<uint16_t>(bytes[index] |
                                 (static_cast<uint16_t>(bytes[index + 1u])
                                  << 8u));
}

uint32_t Le32(const uint8_t* bytes, unsigned index)
{
    return static_cast<uint32_t>(bytes[index]) |
           (static_cast<uint32_t>(bytes[index + 1u]) << 8u) |
           (static_cast<uint32_t>(bytes[index + 2u]) << 16u) |
           (static_cast<uint32_t>(bytes[index + 3u]) << 24u);
}

bool ParseRole(const char* text, unsigned& index)
{
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        if (std::strcmp(text, RoleKeys[i]) == 0)
        {
            index = i;
            return true;
        }
    }
    return false;
}

const char* ParamValue(const char* query, const char* name)
{
    if (query == nullptr)
    {
        return nullptr;
    }

    const size_t nameLength = std::strlen(name);
    const char*  cursor = query;
    while (*cursor != '\0')
    {
        if (std::strncmp(cursor, name, nameLength) == 0 &&
            cursor[nameLength] == '=')
        {
            return cursor + nameLength + 1u;
        }
        const char* next = std::strchr(cursor, '&');
        if (next == nullptr)
        {
            break;
        }
        cursor = next + 1;
    }
    return nullptr;
}

long ParamLong(const char* query, const char* name, long fallback)
{
    const char* value = ParamValue(query, name);
    return value != nullptr ? std::strtol(value, nullptr, 10) : fallback;
}

void ParamText(const char* query, const char* name, char* out, size_t size)
{
    out[0] = '\0';
    const char* value = ParamValue(query, name);
    if (value == nullptr)
    {
        return;
    }

    size_t index = 0u;
    while (value[index] != '\0' && value[index] != '&' && index + 1u < size)
    {
        out[index] = value[index];
        index++;
    }
    out[index] = '\0';
}

void ClearFaults()
{
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        _faults[i] = FaultState();
        Mt6701SlaveSim::SetTrackLoss(Roles[i], false);
        Mt6701SlaveSim::SetCrcBroken(Roles[i], false);
        Mt6701SlaveSim::SetStuck(Roles[i], false);
        Mt6701SlaveSim::SetFieldStatus(Roles[i], 0u);
    }
}

void ApplyFault(const char* roleText, const char* kind, long value)
{
    unsigned index = 0u;
    if (!ParseRole(roleText, index))
    {
        return;
    }

    const bool enabled = value != 0;
    if (std::strcmp(kind, "track") == 0)
    {
        _faults[index].Track = enabled;
        Mt6701SlaveSim::SetTrackLoss(Roles[index], enabled);
    }
    else if (std::strcmp(kind, "crc") == 0)
    {
        _faults[index].Crc = enabled;
        Mt6701SlaveSim::SetCrcBroken(Roles[index], enabled);
    }
    else if (std::strcmp(kind, "stuck") == 0)
    {
        _faults[index].Stuck = enabled;
        Mt6701SlaveSim::SetStuck(Roles[index], enabled);
    }
    else if (std::strcmp(kind, "field") == 0)
    {
        _faults[index].Field = static_cast<uint8_t>(value & 0x03L);
        Mt6701SlaveSim::SetFieldStatus(Roles[index], _faults[index].Field);
    }
}

std::string JsonState()
{
    uint8_t bytes[PositionRegister::RegisterCount];
    positionRegister.Select(0u);
    positionRegister.Read(bytes);

    const uint32_t shaft = Mt6701SlaveSim::ShaftPosition();
    const uint16_t wheelAngles[PositionRegister::WheelCount] = {
        GearTrainSim::SunAngle(shaft),
        GearTrainSim::WheelAngle(shaft, GearConfig::DrivenTeethCounts[0]),
        GearTrainSim::WheelAngle(shaft, GearConfig::DrivenTeethCounts[1]),
        GearTrainSim::WheelAngle(shaft, GearConfig::DrivenTeethCounts[2])};
    const uint32_t count = Le32(bytes, 0u);
    const uint8_t  status = bytes[PositionRegister::RegisterStatus];
    const unsigned counter =
        Le16(bytes, PositionRegister::RegisterStatus + 1u);

    char        buffer[512];
    std::string json;
    json.reserve(1500u);

    std::snprintf(buffer, sizeof buffer,
                  "{\"config\":{\"inputTeeth\":%u,\"turnRange\":%u,"
                  "\"teeth\":[%u,%u,%u]},",
                  static_cast<unsigned>(GearConfig::InputTeeth),
                  static_cast<unsigned>(GearConfig::TurnRange),
                  static_cast<unsigned>(GearConfig::DrivenTeethCounts[0]),
                  static_cast<unsigned>(GearConfig::DrivenTeethCounts[1]),
                  static_cast<unsigned>(GearConfig::DrivenTeethCounts[2]));
    json += buffer;

    std::snprintf(buffer, sizeof buffer,
                  "\"shaft\":{\"count\":%u,\"turns\":%u,\"angle\":%u},",
                  static_cast<unsigned>(shaft),
                  static_cast<unsigned>(shaft / 16384u),
                  static_cast<unsigned>(shaft % 16384u));
    json += buffer;

    json += "\"wheels\":[";
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        std::snprintf(buffer, sizeof buffer, "%s{\"angle\":%u}",
                      i == 0u ? "" : ",",
                      static_cast<unsigned>(wheelAngles[i]));
        json += buffer;
    }
    json += "],";

    json += "\"chips\":[";
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        const unsigned angleRegister =
            PositionRegister::GearAngleRegister(Roles[i]);
        const unsigned diagRegister =
            PositionRegister::DiagnosticsRegister(Roles[i]);
        std::snprintf(buffer, sizeof buffer,
                      "%s{\"angle\":%u,\"cause\":%u}",
                      i == 0u ? "" : ",",
                      static_cast<unsigned>(Le16(bytes, angleRegister)),
                      static_cast<unsigned>(bytes[diagRegister]));
        json += buffer;
    }
    json += "],";

    std::snprintf(buffer, sizeof buffer,
                  "\"position\":{\"count\":%u,\"turns\":%u,\"angle\":%u,"
                  "\"status\":%u,\"valid\":%s},",
                  static_cast<unsigned>(count),
                  static_cast<unsigned>(count / 16384u),
                  static_cast<unsigned>(count % 16384u),
                  static_cast<unsigned>(status),
                  (status & PositionRegister::StatusValid) != 0u ? "true"
                                                                 : "false");
    json += buffer;

    std::snprintf(buffer, sizeof buffer, "\"counter\":%u,",
                  static_cast<unsigned>(counter));
    json += buffer;

    json += "\"diag\":[";
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        const unsigned diagRegister =
            PositionRegister::DiagnosticsRegister(Roles[i]);
        std::snprintf(
            buffer, sizeof buffer,
            "%s{\"cause\":%u,\"mg\":%u,\"frameFails\":%u,\"statusFails\":%u,"
            "\"lastFailSeq\":%u}",
            i == 0u ? "" : ",",
            static_cast<unsigned>(bytes[diagRegister]),
            static_cast<unsigned>(bytes[diagRegister + 1u]),
            static_cast<unsigned>(Le16(bytes, diagRegister + 2u)),
            static_cast<unsigned>(Le16(bytes, diagRegister + 4u)),
            static_cast<unsigned>(Le16(bytes, diagRegister + 6u)));
        json += buffer;
    }
    json += "],";

    json += "\"faults\":[";
    for (unsigned i = 0u; i < PositionRegister::WheelCount; i++)
    {
        std::snprintf(buffer, sizeof buffer,
                      "%s{\"track\":%s,\"crc\":%s,\"stuck\":%s,\"field\":%u}",
                      i == 0u ? "" : ",",
                      _faults[i].Track ? "true" : "false",
                      _faults[i].Crc ? "true" : "false",
                      _faults[i].Stuck ? "true" : "false",
                      static_cast<unsigned>(_faults[i].Field));
        json += buffer;
    }
    json += "]}";

    return json;
}

bool HandleApi(const char* path, const char* query, std::string& body)
{
    if (std::strcmp(path, "/api/state") == 0)
    {
        body = JsonState();
        return true;
    }
    if (std::strcmp(path, "/api/sample") == 0)
    {
        AppProcessSample();
        body = JsonState();
        return true;
    }
    if (std::strcmp(path, "/api/reset") == 0)
    {
        ClearFaults();
        AppReset();
        AppProcessSample();
        body = JsonState();
        return true;
    }
    if (std::strcmp(path, "/api/move") == 0)
    {
        const long counts = ParamLong(query, "counts", 0L);
        Mt6701SlaveSim::AdvanceShaft(static_cast<int32_t>(counts));
        AppProcessSample();
        body = JsonState();
        return true;
    }
    if (std::strcmp(path, "/api/set") == 0)
    {
        long turns = ParamLong(query, "turns", 0L);
        long sun = ParamLong(query, "sun", 0L);
        if (turns < 0L)
        {
            turns = 0L;
        }
        if (sun < 0L)
        {
            sun = 0L;
        }
        turns %= static_cast<long>(GearConfig::TurnRange);
        sun %= 16384L;
        Mt6701SlaveSim::SetShaftPosition(
            static_cast<uint32_t>(turns) * 16384u +
            static_cast<uint32_t>(sun));
        AppProcessSample();
        body = JsonState();
        return true;
    }
    if (std::strcmp(path, "/api/fault") == 0)
    {
        char role[16] = {0};
        char kind[16] = {0};
        ParamText(query, "role", role, sizeof role);
        ParamText(query, "kind", kind, sizeof kind);
        ApplyFault(role, kind, ParamLong(query, "value", 1L));
        AppProcessSample();
        body = JsonState();
        return true;
    }
    return false;
}

void SendAll(SocketHandle client, const std::string& data)
{
    size_t sent = 0u;
    while (sent < data.size())
    {
        const int result =
            send(client, data.data() + sent,
                 static_cast<int>(data.size() - sent), 0);
        if (result <= 0)
        {
            return;
        }
        sent += static_cast<size_t>(result);
    }
}

void SendResponse(SocketHandle client, const char* contentType,
                  const std::string& body)
{
    char header[320];
    const int headerLength = std::snprintf(
        header, sizeof header,
        "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
        "Connection: close\r\nCache-Control: no-store\r\n"
        "Access-Control-Allow-Origin: *\r\n\r\n",
        contentType, static_cast<unsigned>(body.size()));
    SendAll(client, std::string(header, static_cast<size_t>(headerLength)));
    SendAll(client, body);
}

void HandleClient(SocketHandle client)
{
    char request[4096];
    const int received = recv(client, request, sizeof(request) - 1, 0);
    if (received <= 0)
    {
        return;
    }
    request[received] = '\0';

    char method[8] = {0};
    char target[2048] = {0};
    if (std::sscanf(request, "%7s %2047s", method, target) != 2)
    {
        return;
    }

    char* query = nullptr;
    char* questionMark = std::strchr(target, '?');
    if (questionMark != nullptr)
    {
        *questionMark = '\0';
        query = questionMark + 1;
    }

    std::string body;
    const char* contentType = "application/json";
    if (std::strcmp(target, "/") == 0 ||
        std::strcmp(target, "/index.html") == 0)
    {
        body = _page;
        contentType = "text/html; charset=utf-8";
    }
    else if (!HandleApi(target, query, body))
    {
        body = "{\"error\":\"not found\"}";
    }

    SendResponse(client, contentType, body);
}
} // namespace

/* The page is embedded so the binary runs from any directory. */
extern const char* const _page = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Absolute encoder simulation</title>
<style>
:root { color-scheme: dark; }
* { box-sizing: border-box; }
body { margin: 0; background: #1b1d1f; color: #d7dae0; overflow: hidden;
       font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
header { display: flex; align-items: center; gap: 12px;
         padding: 14px 18px; border-bottom: 1px solid #2c3035; }
h1 { margin: 0; font-size: 16px; font-weight: 600; letter-spacing: .02em; }
.pill { margin-left: auto; padding: 3px 10px; border-radius: 999px;
        background: #2a2e33; color: #9aa3ad; font-size: 12px; }
.pill.ok { background: #173b2a; color: #6fcf97; }
.pill.bad { background: #3b1d1d; color: #e06c75; }
main { display: grid; grid-template-columns: minmax(0, 1fr) 390px;
       gap: 14px; padding: 14px; height: calc(100vh - 52px); }
.viewport { position: relative; min-height: 420px; overflow: hidden;
            background: #16181a; border: 1px solid #2c3035; border-radius: 10px; }
canvas { position: absolute; inset: 0; display: block; width: 100%; height: 100%; }
.panel { display: flex; flex-direction: column; gap: 12px; overflow: auto; }
.card { background: #202327; border: 1px solid #2c3035; border-radius: 10px;
        padding: 12px; }
.card-title { margin-bottom: 8px; color: #8b949e; font-size: 12px;
              text-transform: uppercase; letter-spacing: .08em; }
.big { margin-bottom: 8px; color: #e6edf3; font-weight: 600; font-size: 21px;
       font-family: ui-monospace, SFMono-Regular, Consolas, monospace; }
.kv { display: flex; justify-content: space-between; padding: 2px 0;
      color: #9aa3ad; font-size: 13px; }
.kv b { color: #e6edf3; font-weight: 500;
        font-family: ui-monospace, monospace; }
input[type=range] { width: 100%; accent-color: #4ec9b0; }
button { background: #2b3037; color: #d7dae0; border: 1px solid #3a4048;
         border-radius: 6px; padding: 6px 10px; font: inherit; cursor: pointer; }
button:hover { background: #343a42; }
button.on { background: #1f4d3f; border-color: #2f7a63; color: #8be9c6; }
.row { display: flex; gap: 8px; margin-top: 8px; }
.row > * { flex: 1; }
label { display: block; margin-top: 10px; color: #9aa3ad; font-size: 12px; }
table { width: 100%; border-collapse: collapse;
        font-family: ui-monospace, monospace; font-size: 12px; }
th, td { padding: 5px 4px; border-bottom: 1px solid #2a2e33; text-align: left; }
th { color: #8b949e; font-weight: 500; }
td.num { text-align: right; }
.badge { padding: 1px 6px; border-radius: 4px; background: #2a2e33;
         color: #9aa3ad; font-size: 11px; }
.badge.ok { background: #173b2a; color: #6fcf97; }
.badge.bad { background: #3b1d1d; color: #e06c75; }
#faultRows button { padding: 3px 7px; font-size: 11px; }
@media (max-width: 900px) {
  body { overflow: auto; }
  main { grid-template-columns: 1fr; height: auto; }
  .viewport { min-height: 360px; }
}
</style>
</head>
<body>
<header>
  <h1>Absolute encoder simulation</h1>
  <span class="pill" id="conn">connecting&hellip;</span>
</header>
<main>
  <section class="viewport"><canvas id="gears"></canvas></section>
  <aside class="panel">
    <div class="card">
      <div class="card-title">Input shaft</div>
      <div class="big" id="shaftText">&mdash;</div>
      <input id="angle" type="range" min="0" max="16383" step="1" value="0">
      <div class="row">
        <button id="minusTurn">&minus;1 turn</button>
        <button id="plusTurn">+1 turn</button>
      </div>
      <div class="row">
        <button id="play">Play</button>
        <button id="reverse">Reverse</button>
      </div>
      <label>Speed <span id="speedText">800</span> counts / sample</label>
      <input id="speed" type="range" min="50" max="8000" step="50" value="800">
      <div class="row"><button id="resync">Re-sync decoder</button></div>
    </div>
    <div class="card">
      <div class="card-title">Decoded (firmware)</div>
      <div class="big" id="decodedText">&mdash;</div>
      <div class="kv"><span>valid</span><b id="validText">&mdash;</b></div>
      <div class="kv"><span>status</span><b id="statusText">&mdash;</b></div>
      <div class="kv"><span>counter</span><b id="counterText">&mdash;</b></div>
    </div>
    <div class="card">
      <div class="card-title">Chips (measured)</div>
      <table>
        <thead><tr><th>wheel</th><th>angle</th><th>health</th></tr></thead>
        <tbody id="chipRows"></tbody>
      </table>
    </div>
    <div class="card">
      <div class="card-title">Fault injection</div>
      <table>
        <thead><tr><th>wheel</th><th colspan="3">force</th></tr></thead>
        <tbody id="faultRows"></tbody>
      </table>
    </div>
  </aside>
</main>
<script>
"use strict";
const NAMES = ["Sun", "Gear1", "Gear2", "Gear3"];
const KEYS = ["sun", "g1", "g2", "g3"];
const COLORS = ["#e5c07b", "#56b6c2", "#98c379", "#c678dd"];

let state = null;
let playing = false;
let reverse = false;
let speed = 800;
let lastMove = 0;
let moveInFlight = false;

const canvas = document.getElementById("gears");
const ctx = canvas.getContext("2d");

function positionText(turns, angle) {
  return turns + "t + " + String(angle).padStart(5, "0") + "/16384";
}

function setConnected(ok) {
  const pill = document.getElementById("conn");
  pill.className = "pill " + (ok ? "ok" : "bad");
  pill.textContent = ok ? "connected" : "disconnected";
}

async function request(path) {
  try {
    const response = await fetch(path, { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    applyState(await response.json());
    setConnected(true);
  } catch (error) {
    setConnected(false);
    console.error("request failed", path, error);
  }
}

function applyState(next) {
  state = next;
  document.getElementById("shaftText").textContent =
    positionText(next.shaft.turns, next.shaft.angle);
  document.getElementById("angle").value = next.shaft.angle;
  document.getElementById("decodedText").textContent =
    positionText(next.position.turns, next.position.angle);
  const valid = document.getElementById("validText");
  valid.textContent = next.position.valid ? "yes" : "no";
  valid.style.color = next.position.valid ? "#6fcf97" : "#e06c75";
  document.getElementById("statusText").textContent =
    "0x" + next.position.status.toString(16).padStart(2, "0");
  document.getElementById("counterText").textContent = String(next.counter);

  if (document.getElementById("chipRows").children.length !== 4) {
    buildTables(next);
  }
  updateTables(next);
  draw();
}

function buildTables(next) {
  const teeth = [next.config.inputTeeth].concat(next.config.teeth);
  const chipRows = document.getElementById("chipRows");
  const faultRows = document.getElementById("faultRows");
  chipRows.innerHTML = "";
  faultRows.innerHTML = "";
  for (let i = 0; i < 4; i++) {
    chipRows.insertAdjacentHTML(
      "beforeend",
      "<tr><td>" + NAMES[i] + ' <span class="badge">' + teeth[i] + "t</span></td>" +
      '<td class="num" id="chipAngle' + i + '">&mdash;</td>' +
      '<td id="chipHealth' + i + '">&mdash;</td></tr>');
    faultRows.insertAdjacentHTML(
      "beforeend",
      "<tr><td>" + NAMES[i] + "</td>" +
      '<td><button data-i="' + i + '" data-kind="track">track</button></td>' +
      '<td><button data-i="' + i + '" data-kind="crc">crc</button></td>' +
      '<td><button data-i="' + i + '" data-kind="stuck">stuck</button></td></tr>');
  }
  faultRows.onclick = function (event) {
    const button = event.target.closest("button[data-kind]");
    if (!button || !state) return;
    const index = Number(button.dataset.i);
    const kind = button.dataset.kind;
    const on = state.faults[index][kind] ? 0 : 1;
    request("/api/fault?role=" + KEYS[index] + "&kind=" + kind + "&value=" + on);
  };
}

function updateTables(next) {
  next.chips.forEach(function (chip, i) {
    document.getElementById("chipAngle" + i).textContent = String(chip.angle);
    const health = document.getElementById("chipHealth" + i);
    if (chip.cause === 0) {
      health.innerHTML = '<span class="badge ok">ok</span>';
    } else {
      const name = chip.cause === 1 ? "magnetic" : "no-frame";
      health.innerHTML = '<span class="badge bad">' + name + "</span>";
    }
  });
  document.querySelectorAll("#faultRows button").forEach(function (button) {
    const index = Number(button.dataset.i);
    button.classList.toggle(
      "on", Boolean(next.faults[index][button.dataset.kind]));
  });
}

function mod(x, m) { return ((x % m) + m) % m; }

/* Phase a driven gear so its teeth sit in the Sun's gaps along the line of
 * centres: a Sun tooth at the contact (sunPhase 0) must face a gear gap
 * (gearPhase 0.5), i.e. sunPhase + gearPhase == 0.5. */
function meshOffset(alpha, sunTeeth, gearTeeth) {
  const sunPitch = Math.PI * 2 / sunTeeth;
  const gearPitch = Math.PI * 2 / gearTeeth;
  const sunPhase = mod(alpha, sunPitch) / sunPitch;
  const gearPhase = mod(alpha + Math.PI, gearPitch) / gearPitch;
  return mod(sunPhase + gearPhase - 0.5, 1) * gearPitch;
}

function drawGear(x, y, radius, teeth, rotation, color, label, angle,
                  labelAngle) {
  const toothDepth = Math.max(3, radius * 0.10);
  const outer = radius + toothDepth;
  const inner = Math.max(4, radius - toothDepth);
  const steps = teeth * 2;

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(rotation);
  ctx.beginPath();
  for (let i = 0; i < steps; i++) {
    const a = (i / steps) * Math.PI * 2;
    const r = i % 2 === 0 ? outer : inner;
    const px = Math.cos(a) * r;
    const py = Math.sin(a) * r;
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  }
  ctx.closePath();
  ctx.fillStyle = color + "1f";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.fill();
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, 0, Math.max(4, radius * 0.16), 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();

  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(inner * 0.9, 0);
  ctx.strokeStyle = "#e6edf3";
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = "#9aa3ad";
  ctx.font = "12px ui-monospace, SFMono-Regular, Consolas, monospace";
  ctx.textAlign = "center";
  const labelDistance = outer + 24;
  const lx = x + Math.cos(labelAngle) * labelDistance;
  const ly = y + Math.sin(labelAngle) * labelDistance;
  ctx.fillText(label + " \u00b7 " + teeth + "t", lx, ly);
  ctx.fillText(angle + "/16384", lx, ly + 16);
}

function draw() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = rect.width;
  const h = rect.height;
  ctx.clearRect(0, 0, w, h);
  if (!state) return;

  const teeth = [state.config.inputTeeth].concat(state.config.teeth);
  const maxTeeth = Math.max.apply(null, teeth);
  const unit = (Math.min(w, h) * 0.85) / (2 * (teeth[0] + 2 * maxTeeth));
  const cx = w / 2;
  const cy = h / 2;
  const placements = [0, -Math.PI / 2, Math.PI / 6, 5 * Math.PI / 6];

  for (let i = 3; i >= 1; i--) {
    const distance = (teeth[0] + teeth[i]) * unit;
    const x = cx + Math.cos(placements[i]) * distance;
    const y = cy + Math.sin(placements[i]) * distance;
    const rotation = -state.wheels[i].angle / 16384 * Math.PI * 2 +
                     meshOffset(placements[i], teeth[0], teeth[i]);
    drawGear(x, y, teeth[i] * unit, teeth[i], rotation, COLORS[i],
             NAMES[i], state.wheels[i].angle, placements[i]);
  }
  drawGear(cx, cy, teeth[0] * unit, teeth[0],
           state.wheels[0].angle / 16384 * Math.PI * 2, COLORS[0],
           NAMES[0], state.wheels[0].angle, Math.PI / 2);
}

function syncPlay() {
  const button = document.getElementById("play");
  button.textContent = playing ? "Pause" : "Play";
  button.classList.toggle("on", playing);
}

function syncReverse() {
  document.getElementById("reverse").classList.toggle("on", reverse);
}

document.getElementById("angle").addEventListener("input", function (event) {
  if (!state) return;
  playing = false;
  syncPlay();
  request("/api/set?turns=" + state.shaft.turns + "&sun=" + event.target.value);
});
document.getElementById("minusTurn").addEventListener("click",
  function () { request("/api/move?counts=-16384"); });
document.getElementById("plusTurn").addEventListener("click",
  function () { request("/api/move?counts=16384"); });
document.getElementById("play").addEventListener("click", function () {
  playing = !playing;
  syncPlay();
});
document.getElementById("reverse").addEventListener("click", function () {
  reverse = !reverse;
  syncReverse();
});
document.getElementById("speed").addEventListener("input", function (event) {
  speed = Number(event.target.value);
  document.getElementById("speedText").textContent = String(speed);
});
document.getElementById("resync").addEventListener("click",
  function () { request("/api/reset"); });

function tick(now) {
  requestAnimationFrame(tick);
  draw();
  if (!playing || moveInFlight) return;
  if (now - lastMove < 40) return;
  lastMove = now;
  moveInFlight = true;
  const counts = speed * (reverse ? -1 : 1);
  request("/api/move?counts=" + counts).finally(function () {
    moveInFlight = false;
  });
}

request("/api/sample");
requestAnimationFrame(tick);
</script>
</body>
</html>)HTML";

int main(int argc, char** argv)
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::printf("WSAStartup failed\n");
        return 1;
    }
#else
    signal(SIGPIPE, SIG_IGN);
#endif

    int port = 8080;
    if (argc > 1)
    {
        port = std::atoi(argv[1]);
    }

    AppInit();

    const SocketHandle listener = socket(AF_INET, SOCK_STREAM, 0);
    if (listener == INVALID_SOCKET)
    {
        std::printf("socket() failed\n");
        return 1;
    }

    int reuse = 1;
#ifdef _WIN32
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&reuse), sizeof reuse);
#else
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);
#endif

    sockaddr_in address;
    std::memset(&address, 0, sizeof address);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<unsigned short>(port));

    if (bind(listener, reinterpret_cast<sockaddr*>(&address),
             sizeof address) != 0 ||
        listen(listener, 16) != 0)
    {
        std::printf("bind/listen failed on port %d\n", port);
        return 1;
    }

    std::printf("encoder GUI: http://127.0.0.1:%d/\n", port);
    std::fflush(stdout);

    for (;;)
    {
        const SocketHandle client = accept(listener, nullptr, nullptr);
        if (client == INVALID_SOCKET)
        {
            continue;
        }
        HandleClient(client);
        CLOSE_SOCKET(client);
    }
}
