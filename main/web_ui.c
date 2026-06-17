/**
 * Web Diagnostics and Configuration UI
 */

#include "web_ui.h"

#include "ble_service.h"
#include "can_driver.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "exlap_client.h"
#include "telemetry_db.h"
#include "tirex_config.h"
#include "wifi_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "lwip/inet.h"

static const char *TAG = "web_ui";
static httpd_handle_t s_httpd = NULL;

#if CONFIG_EXLAP_TESTING_AP_UI
static const char s_index_html[] =
"<!doctype html><html lang=\"en\"><head>"
"<meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>EXLAP Gateway</title>"
"<style>"
"body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:#0f172a;color:#e2e8f0;}"
".wrap{max-width:1100px;margin:0 auto;padding:24px;}"
".hero{display:flex;flex-wrap:wrap;gap:12px;align-items:end;justify-content:space-between;padding:24px;border:1px solid #1e293b;border-radius:20px;background:linear-gradient(135deg,#111827,#0f172a 55%,#1e293b);}"
".hero h1{margin:0;font-size:28px;}"
".hero p{margin:8px 0 0;color:#94a3b8;}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:16px;margin-top:16px;}"
".card{background:#111827;border:1px solid #1e293b;border-radius:18px;padding:16px;box-shadow:0 12px 32px rgba(0,0,0,.22);}"
".card h2{margin:0 0 12px;font-size:16px;color:#f8fafc;}"
".stat{display:flex;justify-content:space-between;gap:12px;padding:6px 0;border-bottom:1px solid rgba(148,163,184,.12);font-size:14px;}"
".stat:last-child{border-bottom:none;}"
".label{color:#94a3b8;}"
".value{font-variant-numeric:tabular-nums;text-align:right;}"
".pill{display:inline-block;padding:4px 10px;border-radius:999px;background:#1f2937;color:#d1fae5;border:1px solid #334155;font-size:12px;}"
".form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;}"
"label{display:block;font-size:12px;color:#94a3b8;margin-bottom:6px;}"
"input{width:100%;box-sizing:border-box;background:#0b1220;color:#e2e8f0;border:1px solid #334155;border-radius:12px;padding:10px 12px;font-size:14px;}"
".row{display:flex;gap:12px;align-items:center;flex-wrap:wrap;}"
"button{background:#38bdf8;color:#08111f;border:none;border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;}"
"button:hover{filter:brightness(1.05);}"
".muted{color:#94a3b8;font-size:12px;}"
".good{color:#86efac;}"
".bad{color:#fca5a5;}"
"pre{white-space:pre-wrap;word-break:break-word;background:#0b1220;border:1px solid #334155;border-radius:14px;padding:12px;overflow:auto;}"
"</style></head><body>"
"<div class=\"wrap\">"
"<div class=\"hero\">"
"<div><h1>EXLAP Gateway</h1><p>Live status, telemetry, and configuration for the board.</p></div>"
"<div class=\"pill\" id=\"uptime\">booting...</div>"
"</div>"
"<div class=\"grid\">"
"<div class=\"card\"><h2>System</h2><div id=\"sys\"></div></div>"
"<div class=\"card\"><h2>Connection</h2><div id=\"conn\"></div></div>"
"<div class=\"card\"><h2>Telemetry</h2><div id=\"telemetry\"></div></div>"
"<div class=\"card\"><h2>Diagnostics</h2><div id=\"diag\"></div></div>"
"</div>"
"<div class=\"card\" style=\"margin-top:16px;\">"
"<h2>TireX Configuration</h2><div id=\"tirex\">loading...</div>"
"</div>"
"<div class=\"card\" style=\"margin-top:16px;\">"
"<h2>WiFi Configuration</h2>"
"<form id=\"wifiForm\">"
"<div class=\"form-grid\">"
"<div><label>SSID</label><input name=\"wifi_ssid\" id=\"wifi_ssid\" placeholder=\"Porsche_WLAN_...\"></div>"
"<div><label>Password</label><input name=\"wifi_password\" id=\"wifi_password\" type=\"password\" placeholder=\"car wifi password\"></div>"
"</div>"
"<div class=\"row\" style=\"margin-top:14px;\">"
"<button type=\"submit\">Save WiFi</button><button type=\"button\" id=\"wifiConnectBtn\">Join Car WiFi (pause AP)</button><button type=\"button\" id=\"wifiDisconnectBtn\">Stop WiFi</button><span class=\"muted\" id=\"wifiSaveState\"></span>"
"</div>"
"</form>"
"</div>"
"<div class=\"card\" style=\"margin-top:16px;\">"
"<h2>EXLAP Configuration</h2>"
"<form id=\"cfgForm\">"
"<div class=\"form-grid\">"
"<div><label>Vehicle IP</label><input name=\"vehicle_ip\" id=\"vehicle_ip\" placeholder=\"192.168.0.10\"></div>"
"<div><label>Port</label><input name=\"port\" id=\"port\" type=\"number\" min=\"1\" max=\"65535\"></div>"
"<div><label>Username</label><input name=\"username\" id=\"username\"></div>"
"<div><label>Password</label><input name=\"password\" id=\"password\" type=\"password\"></div>"
"<div><label>Enabled</label><input name=\"enabled\" id=\"enabled\" type=\"checkbox\" style=\"width:auto;margin-top:11px\"></div>"
"</div>"
"<div class=\"row\" style=\"margin-top:14px;\">"
"<button type=\"submit\">Save Config</button><span class=\"muted\" id=\"saveState\"></span>"
"</div>"
"</form>"
"</div>"
"<div class=\"card\" style=\"margin-top:16px;\">"
"<h2>Raw Telemetry</h2><pre id=\"raw\">loading...</pre>"
"</div>"
"</div>"
"<script>"
"const fmt = (v) => (v === null || v === undefined || v === '' ? 'n/a' : v);"
"const kv = (k,v,c='') => `<div class=\"stat\"><div class=\"label\">${k}</div><div class=\"value ${c}\">${fmt(v)}</div></div>`;"
"const htmlEscape = (s) => String(s ?? '').replace(/[&<>\"']/g, (ch) => { if (ch === '&') return '&amp;'; if (ch === '<') return '&lt;'; if (ch === '>') return '&gt;'; if (ch.charCodeAt(0) === 34) return '&quot;'; return '&#39;'; });"
"const cornerLabel = (v) => ({0:'left front',1:'right front',2:'left rear',3:'right rear',255:'unknown'}[v] ?? 'unknown');"
"let tirexDirty = false;"
"const tirexHeatColdC = 28.0;"
"const tirexHeatHotC = 34.0;"
"const heatColor = (v) => { const t = Math.max(0, Math.min(1, (Number(v) - tirexHeatColdC) / (tirexHeatHotC - tirexHeatColdC))); const hue = 240 - (t * 240); const sat = 100; const light = 52; return `hsl(${hue},${sat}%,${light}%)`; };"
"const tirexZoneOneEdge = (corner) => { const isLeftSide = Number(corner) === 0 || Number(corner) === 2; return isLeftSide ? 'right edge (toward vehicle centerline)' : 'left edge (toward vehicle centerline)'; };"
"const tirexFrontEdge = () => 'top edge';"
"const tirexFrameCols = 16;"
"const tirexFrameRows = 12;"
"const tirexPairCols = 8;"
"const tirexPairRows = 6;"
"const tirexLiveTargetHz = 64;"
"const tirexLiveIntervalMs = 1000 / tirexLiveTargetHz;"
"let tirexLiveTimer = null;"
"let tirexLiveBusy = false;"
"function tirexFrameMax(cells){ let max = 1; for (const v of cells) { if (v > max) max = v; } return max; }"
"function renderHeatmap(cells, cols = tirexFrameCols, live = false){"
"if (!cells || !cells.length) return '<div class=\"muted\">No full-frame pixels captured yet.</div>';"
"let out = `<div style=\"display:grid;grid-template-columns:repeat(${cols},minmax(0,1fr));gap:2px;max-width:100%;\">`;"
"for (let i=0;i<cells.length;i++){ const v = Number(cells[i] ?? 0); const liveAttrs = live ? ` class=\"tirexLivePixel\" data-pixel-index=\"${i}\"` : ''; out += `<div${liveAttrs} title=\"${v.toFixed(1)}\" style=\"aspect-ratio:1;background:${heatColor(v)};border-radius:3px;\"></div>`; }"
"out += '</div>'; return out;"
"}"
"function renderHeatSummary(cells){"
" if (!cells || !cells.length) return '<div class=\"muted\">No frame summary yet.</div>';"
" let min = Number.POSITIVE_INFINITY; let max = Number.NEGATIVE_INFINITY; let sum = 0;"
" for (const raw of cells) { const v = Number(raw ?? 0); if (v < min) min = v; if (v > max) max = v; sum += v; }"
" const avg = sum / cells.length;"
" return `<div class=\"muted\" style=\"margin-top:6px;\">avg ${avg.toFixed(1)} | min ${min.toFixed(1)} | max ${max.toFixed(1)} | span ${(max - min).toFixed(1)}</div>`;"
"}"
"function renderHeatLegend(){"
" return `<div style=\"display:flex;align-items:center;gap:10px;margin-top:8px;\"><div class=\"muted\" style=\"font-size:12px;\">${tirexHeatColdC.toFixed(0)} C</div><div style=\"flex:1;height:10px;border-radius:999px;background:linear-gradient(90deg,#08162e 0%,#0f3f91 30%,#3b82f6 52%,#f97316 76%,#dc2626 100%);border:1px solid rgba(148,163,184,.25);\"></div><div class=\"muted\" style=\"font-size:12px;\">${tirexHeatHotC.toFixed(0)} C</div></div>`;"
"}"
"function mapFrame(frame, cols, rows, mapper){"
" const cells = [];"
" for (let row = 0; row < rows; row++) {"
"  for (let col = 0; col < cols; col++) {"
"   const idx = mapper(row, col);"
"   const v = idx >= 0 && idx < frame.length ? Number(frame[idx] ?? 0) : 0;"
"   cells.push(v);"
"  }"
" }"
" return cells;"
"}"
"function binFrameHorizontalPairs(frame){"
" const out = [];"
" for (let row = 0; row < tirexFrameRows; row++) {"
"  for (let col = 0; col < tirexPairCols; col++) {"
"   const idx = row * tirexFrameCols + (col * 2);"
"   const a = Number(frame[idx] ?? 0);"
"   const b = Number(frame[idx + 1] ?? 0);"
"   out.push((a + b) / 2);"
"  }"
" }"
" return out;"
"}"
"function mapPacketHalves(frame){"
" const out = [];"
" for (let row = 0; row < tirexFrameRows; row++) {"
"  const leftPacket = row * 8;"
"  const rightPacket = (row + tirexFrameRows) * 8;"
"  for (let col = 0; col < 8; col++) out.push(Number(frame[leftPacket + col] ?? 0));"
"  for (let col = 0; col < 8; col++) out.push(Number(frame[rightPacket + col] ?? 0));"
" }"
" return out;"
"}"
"function averagePacketBlocks(frame){"
" const out = [];"
" for (let i = 0; i < 96; i++) {"
"  out.push((Number(frame[i] ?? 0) + Number(frame[i + 96] ?? 0)) / 2);"
" }"
" return out;"
"}"
"function expandHorizontalPairs(frame8x12){"
" const out = [];"
" for (const raw of frame8x12) {"
"  const v = Number(raw ?? 0);"
"  out.push(v, v);"
" }"
" return out;"
"}"
"function renderHeatmapView(title, cols, rows, cells, note){"
" return `<div class=\"card\" style=\"padding:12px;min-width:0;\">"
"  <div style=\"display:flex;justify-content:space-between;gap:12px;align-items:baseline;\">"
"   <div style=\"font-weight:700;color:#f8fafc;\">${htmlEscape(title)}</div>"
"   <div class=\"muted\">${cols} x ${rows}</div>"
"  </div>"
"  <div style=\"margin-top:8px;\">${renderHeatmap(cells, cols)}</div>"
"  ${renderHeatSummary(cells)}"
"  <div class=\"muted\" style=\"margin-top:4px;\">${htmlEscape(note)}</div>"
" </div>`;"
"}"
"function renderFullFrameExperiment(frame, channels){"
" if (!frame || !frame.length || !channels || !channels.length) return '<div class=\"muted\">No mapping experiment data yet.</div>';"
" const sequential = frame.slice();"
" const hflip = mapFrame(frame, tirexFrameCols, tirexFrameRows, (row, col) => row * tirexFrameCols + (tirexFrameCols - 1 - col));"
" const vflip = mapFrame(frame, tirexFrameCols, tirexFrameRows, (row, col) => (tirexFrameRows - 1 - row) * tirexFrameCols + col);"
" const transpose = mapFrame(frame, tirexFrameRows, tirexFrameCols, (row, col) => col * tirexFrameCols + row);"
" const rawChannels = channels.slice();"
" const rawChannelsFlip = mapFrame(channels, tirexPairCols, tirexFrameRows, (row, col) => row * tirexPairCols + (tirexPairCols - 1 - col));"
" return `<div style=\"margin-top:14px;\">"
"  <div class=\"muted\" style=\"margin-bottom:8px;\">TireX reports an 8 x 12 temperature-channel grid. Each channel is expanded into two horizontal pixels for the documented 16 x 12 physical view.</div>"
"  <div class=\"grid\" style=\"grid-template-columns:repeat(auto-fit,minmax(220px,1fr));margin-bottom:14px;\">"
"   ${renderHeatmapView('Raw channel grid', tirexPairCols, tirexFrameRows, rawChannels, '12 packets by 8 temperature channels. Outside-to-centerline motion should travel left-to-right.')} "
"   ${renderHeatmapView('Raw channels flipped', tirexPairCols, tirexFrameRows, rawChannelsFlip, 'Mirrored raw grid for installations where the sensor is physically reversed.')} "
"   ${renderHeatmapView('Physical pixel view', tirexFrameCols, tirexFrameRows, sequential, 'Each reported channel expanded into two adjacent horizontal pixels.')} "
"  </div>"
"  <div class=\"grid\" style=\"grid-template-columns:repeat(auto-fit,minmax(220px,1fr));\">"
"   ${renderHeatmapView('Horizontal flip', tirexFrameCols, tirexFrameRows, hflip, 'Useful if the sensor is mirrored left/right.')} "
"   ${renderHeatmapView('Vertical flip', tirexFrameCols, tirexFrameRows, vflip, 'Useful if the sensor is upside down or top/bottom swapped.')} "
"   ${renderHeatmapView('Transpose', tirexFrameRows, tirexFrameCols, transpose, 'Useful if rows and columns are swapped in the current parser.')} "
"  </div>"
" </div>`;"
"}"
"function markTirexDirty(evt){"
" tirexDirty = true;"
" const form = evt && evt.currentTarget ? evt.currentTarget : null;"
" if (form) { form.dataset.dirty = '1'; }"
"}"
"function clearTirexDirty(){ tirexDirty = false; }"
"function renderTirexCard(s){"
" const hasObservedState = Number(s.last_announce_ms) > 0 || Number(s.last_config_ms) > 0;"
" const displayCorner = hasObservedState ? Number(s.observed_position) : Number(s.corner);"
" const displayZoneCount = hasObservedState ? Number(s.observed_zone_count) : Number(s.zone_count);"
" const displayFullFrame = hasObservedState ? !!s.observed_full_frame_mode : !!s.full_frame_mode;"
" const traceEnabled = !!s.full_frame_trace_enabled;"
" const displayRate = Number(s.sample_rate_code);"
" const zones = (s.zones || []).map((z, i) => `<div class=\"stat\"><div class=\"label\">Zone ${i+1}</div><div class=\"value\">${Number(z).toFixed(1)} C</div></div>`).join('');"
" const frame = renderHeatmap(s.frame_pixels || [], tirexFrameCols, true);"
" const showFrame = displayFullFrame;"
" return `<div class=\"card tirexCard\" data-base-id=\"${s.base_id}\"><form class=\"tirexForm\" data-base-id=\"${s.base_id}\">"
" <div class=\"stat\"><div class=\"label\">Sensor</div><div class=\"value\"><strong>${htmlEscape(s.name || 'unnamed')}</strong></div></div>"
" <div class=\"stat\"><div class=\"label\">Base ID</div><div class=\"value\">0x${Number(s.base_id).toString(16).toUpperCase()}</div></div>"
" <div class=\"stat\"><div class=\"label\">Corner</div><div class=\"value\">${htmlEscape(cornerLabel(displayCorner))}</div></div>"
" <div class=\"stat\"><div class=\"label\">Last seen</div><div class=\"value\">${fmt(s.last_seen_ms)} ms</div></div>"
" <div class=\"stat\"><div class=\"label\">Source</div><div class=\"value\">${hasObservedState ? 'observed CAN state' : 'saved profile'}</div></div>"
" <div class=\"form-grid\" style=\"margin-top:10px;\">"
"  <div><label>Name</label><input name=\"name\" value=\"${htmlEscape(s.name || '')}\"></div>"
"  <div><label>Corner</label><select name=\"corner\">${[0,1,2,3].map(v => `<option value=\"${v}\" ${Number(displayCorner)===v?'selected':''}>${cornerLabel(v)}</option>`).join('')}</select></div>"
"  <div><label>Zone Count</label><select name=\"zone_count\">${[1,2,4,8,16].map(v => `<option value=\"${v}\" ${Number(displayZoneCount)===v?'selected':''}>${v}</option>`).join('')}</select></div>"
"  <div><label>Sample Rate</label><select name=\"sample_rate_code\">${[1,2,4,8,16,32,64].map(v => `<option value=\"${v}\" ${Number(displayRate)===v?'selected':''}>${v} Hz</option>`).join('')}</select></div>"
" </div>"
" <div class=\"row\" style=\"margin-top:12px;\">"
"  <label style=\"display:flex;align-items:center;gap:8px;margin:0;\"><input type=\"checkbox\" name=\"flip_orientation\" ${s.flip_orientation ? 'checked' : ''} style=\"width:auto;margin:0;\">Flip orientation</label>"
"  <label style=\"display:flex;align-items:center;gap:8px;margin:0;\"><input type=\"checkbox\" name=\"full_frame_mode\" ${displayFullFrame ? 'checked' : ''} style=\"width:auto;margin:0;\">Full frame mode</label>"
"  <label style=\"display:flex;align-items:center;gap:8px;margin:0;\"><input type=\"checkbox\" name=\"full_frame_trace_enabled\" ${traceEnabled ? 'checked' : ''} style=\"width:auto;margin:0;\">Trace full frame</label>"
"  <button type=\"submit\">Write / Apply</button>"
" </div>"
" <div class=\"muted\" style=\"margin-top:8px;\">Observed position: ${fmt(s.observed_position)} | Observed zones: ${fmt(s.observed_zone_count)} | Observed rate: ${fmt(s.sample_rate_code)} Hz | Mode: ${s.observed_full_frame_mode ? 'full frame' : 'zone broadcast'}</div>"
" <div class=\"muted\" style=\"margin-top:6px;\">Configured position: ${fmt(s.corner)} | Configured zones: ${fmt(s.zone_count)} | Configured rate: ${fmt(s.sample_rate_code)} Hz | Mode: ${s.full_frame_mode ? 'full frame' : 'zone broadcast'} | Trace: ${traceEnabled ? 'on' : 'off'}</div>"
" <div class=\"muted\" style=\"margin-top:6px;\">Apply status: ${s.awaiting_config_confirm ? 'waiting for CAN confirmation' : 'confirmed / in sync'}</div>"
" <div class=\"muted\" style=\"margin-top:6px;\">Full-frame packets: ${fmt(s.full_frame_packet_count)} | Complete frames: ${fmt(s.full_frame_complete_count)} | Discarded frames: ${fmt(s.full_frame_discarded_count)}</div>"
" <div class=\"muted\" style=\"margin-top:6px;\">Frame sync: ${s.full_frame_synced ? 'synchronized' : (s.full_frame_sync_armed ? 'assembling / resynchronizing' : 'waiting for Write / Apply')} | Assembly: ${fmt(s.full_frame_packets_collected)} / 12 packets | Last complete frame: ${fmt(s.last_full_frame_ms)} ms</div>"
" <div class=\"muted tirexLiveState\" style=\"margin-top:6px;\">Heatmap refresh target: ${tirexLiveTargetHz} Hz</div>"
" <div class=\"muted\" style=\"margin-top:6px;\">Zone 1 / inboard side: ${htmlEscape(tirexZoneOneEdge(displayCorner))} | Front of car: ${htmlEscape(tirexFrontEdge())}</div>"
" <div style=\"margin-top:12px;\">${zones || '<div class=\"muted\">No zone data yet.</div>'}</div>"
" <div style=\"margin-top:12px;\">${showFrame ? (s.has_full_frame_data ? `${frame}${renderHeatLegend()}${renderFullFrameExperiment(s.frame_pixels || [], s.channel_pixels || [])}` : '<div class=\"muted\">Full frame mode enabled, waiting for pixel data...</div>') : '<div class=\"muted\">Full frame mode off, matrix hidden.</div>'}</div>"
" </form></div>`;"
"}"
"function attachTirexFormHandlers(){"
" document.querySelectorAll('.tirexForm').forEach((form)=>{"
"  if (form.dataset.bound === '1') { return; }"
"  form.dataset.bound = '1';"
"  if (!form.dataset.dirty) { form.dataset.dirty = '0'; }"
"  form.addEventListener('input', markTirexDirty);"
"  form.addEventListener('change', markTirexDirty);"
"  form.addEventListener('focusin', markTirexDirty);"
"  form.addEventListener('pointerdown', markTirexDirty);"
"  form.addEventListener('mousedown', markTirexDirty);"
"  form.addEventListener('keydown', markTirexDirty);"
"  form.addEventListener('submit', async (e)=>{"
"   e.preventDefault();"
"   const body = new URLSearchParams();"
"   body.set('base_id', form.dataset.baseId);"
"   body.set('name', form.querySelector('[name=name]').value);"
"   body.set('corner', form.querySelector('[name=corner]').value);"
"   body.set('zone_count', form.querySelector('[name=zone_count]').value);"
"   body.set('sample_rate_code', form.querySelector('[name=sample_rate_code]').value);"
"   body.set('flip_orientation', form.querySelector('[name=flip_orientation]').checked ? '1' : '0');"
"   body.set('full_frame_mode', form.querySelector('[name=full_frame_mode]').checked ? '1' : '0');"
"   body.set('full_frame_trace_enabled', form.querySelector('[name=full_frame_trace_enabled]').checked ? '1' : '0');"
"   body.set('apply', '1');"
"   const rr = await fetch('/api/tirex/config', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});"
"   if (rr.ok) { form.dataset.dirty = '0'; clearTirexDirty(); await loadTirex(); await loadStatus(); }"
"  });"
" });"
"}"
"function renderTirex(t){"
"if (!t || !t.sensors) return '<div class=\"muted\">No TireX sensors discovered yet.</div>';"
"return '<div class=\"grid\">' + t.sensors.map(renderTirexCard).join('') + '</div>';"
"}"
"function refreshTirexCards(t){"
" const container = document.getElementById('tirex');"
" if (!container) return;"
" if (!t || !t.sensors) { container.innerHTML = '<div class=\"muted\">No TireX sensors discovered yet.</div>'; return; }"
" const cards = new Map(Array.from(container.querySelectorAll('.tirexCard')).map((card) => [card.dataset.baseId, card]));"
" const hasDirtyCards = Array.from(container.querySelectorAll('.tirexForm')).some((form) => form.dataset.dirty === '1');"
" let needsFullRender = cards.size === 0;"
" for (const s of t.sensors) {"
"  const key = String(s.base_id);"
"  const card = cards.get(key);"
"  if (!card) { needsFullRender = true; continue; }"
"  const form = card.querySelector('.tirexForm');"
"  if (form && form.dataset.dirty === '1') { continue; }"
"  card.outerHTML = renderTirexCard(s);"
" }"
" if (needsFullRender && !hasDirtyCards) { container.innerHTML = renderTirex(t); }"
" attachTirexFormHandlers();"
"}"
"async function loadTirex(){const r=await fetch('/api/tirex'); const j=await r.json(); clearTirexDirty(); document.getElementById('tirex').innerHTML = renderTirex(j); attachTirexFormHandlers(); }"
"function writeTirexLivePixels(card, channels){"
" const cells = card.querySelectorAll('.tirexLivePixel');"
" if (cells.length !== tirexFrameCols * tirexFrameRows) return;"
" for (let channel = 0; channel < channels.length; channel++) {"
"  const value = Number(channels[channel]);"
"  const pixel = channel * 2;"
"  for (let copy = 0; copy < 2; copy++) {"
"   const cell = cells[pixel + copy];"
"   if (!cell) continue;"
"   cell.style.background = heatColor(value);"
"   cell.title = value.toFixed(1);"
"  }"
" }"
"}"
"async function updateTirexLiveHeatmaps(){"
" if (tirexLiveBusy) return;"
" tirexLiveBusy = true;"
" const started = performance.now();"
" try {"
"  if (!document.hidden) {"
"   const cards = Array.from(document.querySelectorAll('.tirexCard'));"
"   for (const card of cards) {"
"    if (!card.querySelector('.tirexLivePixel')) continue;"
"    const response = await fetch(`/api/tirex/frame?base_id=${encodeURIComponent(card.dataset.baseId)}`, {cache:'no-store'});"
"    if (!response.ok) continue;"
"    const payload = await response.arrayBuffer();"
"    if (payload.byteLength !== 108) continue;"
"    const view = new DataView(payload);"
"    const frameCount = view.getUint32(0, true);"
"    const lastFrameMs = view.getUint32(4, true);"
"    const synced = view.getUint8(8) !== 0;"
"    if (String(frameCount) === card.dataset.liveFrameCount) continue;"
"    card.dataset.liveFrameCount = String(frameCount);"
"    writeTirexLivePixels(card, new Uint8Array(payload, 12, 96));"
"    const state = card.querySelector('.tirexLiveState');"
"    const now = performance.now();"
"    const previousFrameAt = Number(card.dataset.liveFrameAt || 0);"
"    const measuredHz = previousFrameAt > 0 ? 1000 / Math.max(1, now - previousFrameAt) : 0;"
"    card.dataset.liveFrameAt = String(now);"
"    if (state) state.textContent = `Heatmap: ${synced ? 'live' : 'resyncing'} | frame ${frameCount} | ${measuredHz.toFixed(1)} Hz observed | sensor ms ${lastFrameMs}`;"
"   }"
"  }"
" } catch (_) {"
"  /* The 1 Hz status path reports connection failures; keep the live painter quiet. */"
" } finally {"
"  tirexLiveBusy = false;"
"  const elapsed = performance.now() - started;"
"  tirexLiveTimer = setTimeout(updateTirexLiveHeatmaps, Math.max(0, tirexLiveIntervalMs - elapsed));"
" }"
"}"
"async function loadCfg(){const r=await fetch('/api/config'); const j=await r.json();"
"document.getElementById('wifi_ssid').value=j.wifi_ssid||'';"
"document.getElementById('wifi_password').value=j.wifi_password||'';"
"document.getElementById('vehicle_ip').value=j.vehicle_ip||'';"
"document.getElementById('port').value=j.port||'';"
"document.getElementById('username').value=j.username||'';"
"document.getElementById('password').value=j.password||'';"
"document.getElementById('enabled').checked=!!j.enabled;}"
"async function loadStatus(){const [sr,tr] = await Promise.all([fetch('/api/status'), fetch('/api/tirex')]); const j=await sr.json(); const t=await tr.json();"
"document.getElementById('uptime').textContent=`uptime ${Math.floor(j.uptime_ms/1000)}s`;"
"document.getElementById('sys').innerHTML="
"kv('WiFi', j.wifi.state) + kv('SSID', j.wifi.ssid) + kv('RSSI', j.wifi.rssi + ' dBm') + kv('Channel', j.wifi.channel) + kv('Auth', j.wifi.auth_mode) + kv('CAN', j.can.running ? 'running' : 'stopped', j.can.running ? 'good' : 'bad') + kv('BLE', j.ble.connected ? 'connected' : 'idle', j.ble.connected ? 'good' : 'bad');"
"document.getElementById('conn').innerHTML="
"kv('EXLAP state', j.exlap.state) + kv('Authenticated', j.exlap.authenticated ? 'yes' : 'no', j.exlap.authenticated ? 'good' : 'bad') + kv('Packets', j.exlap.packets_received) + kv('Dropped', j.exlap.packets_dropped) + kv('Bytes', j.exlap.bytes_received) + kv('Last packet', j.exlap.last_packet_ms + ' ms') + kv('Last auth', j.exlap.last_auth_ms + ' ms') + kv('Heartbeat', j.exlap.last_heartbeat_ms + ' ms');"
"document.getElementById('telemetry').innerHTML="
"kv('Speed', j.telemetry.vehicle_speed_kmh + ' km/h') + kv('RPM', j.telemetry.engine_rpm) + kv('Throttle', j.telemetry.throttle_pct + ' %') + kv('Brake', j.telemetry.brake_pressure_bar + ' bar') + kv('Steering', j.telemetry.steering_angle_deg + ' deg') + kv('Yaw', j.telemetry.yaw_rate + ' deg/s') + kv('Gear', j.telemetry.gear) + kv('Temps LF/RF/LR/RR', j.telemetry.avg_temp_lf + ' / ' + j.telemetry.avg_temp_rf + ' / ' + j.telemetry.avg_temp_lr + ' / ' + j.telemetry.avg_temp_rr + ' C');"
"document.getElementById('diag').innerHTML="
"kv('Telemetry rate', j.diagnostics.telemetry_rate_hz + ' Hz') + kv('TireX updates', j.telemetry.tirex_update_count) + kv('EXLAP updates', j.telemetry.exlap_update_count) + kv('Last TireX', j.telemetry.last_tirex_ms + ' ms') + kv('Last EXLAP', j.telemetry.last_exlap_ms + ' ms');"
"refreshTirexCards(t);"
"document.getElementById('raw').textContent=JSON.stringify(j,null,2);"
"}"
"document.getElementById('wifiForm').addEventListener('submit', async (e)=>{e.preventDefault();"
"const body = new URLSearchParams();"
"body.set('wifi_ssid', document.getElementById('wifi_ssid').value);"
"body.set('wifi_password', document.getElementById('wifi_password').value);"
"const r = await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});"
"document.getElementById('wifiSaveState').textContent = r.ok ? 'saved' : 'save failed';"
"if (r.ok) { await loadCfg(); await loadStatus(); }"
"});"
"document.getElementById('wifiConnectBtn').addEventListener('click', async ()=>{"
"const body = new URLSearchParams();"
"body.set('wifi_ssid', document.getElementById('wifi_ssid').value);"
"body.set('wifi_password', document.getElementById('wifi_password').value);"
"const r = await fetch('/api/wifi/connect', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});"
"document.getElementById('wifiSaveState').textContent = r.ok ? 'join requested; AP paused' : 'join failed';"
"if (r.ok) { await loadStatus(); }"
"});"
"document.getElementById('wifiDisconnectBtn').addEventListener('click', async ()=>{"
"const r = await fetch('/api/wifi/disconnect', {method:'POST'});"
"document.getElementById('wifiSaveState').textContent = r.ok ? 'wifi stopped' : 'stop failed';"
"if (r.ok) { await loadStatus(); }"
"});"
"document.getElementById('cfgForm').addEventListener('submit', async (e)=>{e.preventDefault();"
"const body = new URLSearchParams();"
"body.set('wifi_ssid', document.getElementById('wifi_ssid').value);"
"body.set('wifi_password', document.getElementById('wifi_password').value);"
"body.set('vehicle_ip', document.getElementById('vehicle_ip').value);"
"body.set('port', document.getElementById('port').value);"
"body.set('username', document.getElementById('username').value);"
"body.set('password', document.getElementById('password').value);"
"body.set('enabled', document.getElementById('enabled').checked ? '1' : '0');"
"const r = await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});"
"document.getElementById('saveState').textContent = r.ok ? 'saved' : 'save failed';"
"if (r.ok) { await loadCfg(); await loadStatus(); }"
"});"
"loadCfg(); loadStatus(); setInterval(loadStatus, 1000); updateTirexLiveHeatmaps();"
"</script></body></html>";

static const char *wifi_state_to_str(wifi_state_t state)
{
    switch (state) {
    case WIFI_STATE_CONNECTING: return "connecting";
    case WIFI_STATE_CONNECTED: return "connected";
    case WIFI_STATE_DISCONNECTED:
    default:
        return "disconnected";
    }
}

static const char *exlap_state_to_str(exlap_state_t state)
{
    switch (state) {
    case EXLAP_STATE_CONNECTING: return "connecting";
    case EXLAP_STATE_AUTHENTICATING: return "authenticating";
    case EXLAP_STATE_SUBSCRIBING: return "subscribing";
    case EXLAP_STATE_AUTHENTICATED: return "authenticated";
    case EXLAP_STATE_DISCONNECTED:
    default:
        return "disconnected";
    }
}

static const char *auth_mode_to_str(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN: return "open";
    case WIFI_AUTH_WEP: return "wep";
    case WIFI_AUTH_WPA_PSK: return "wpa-psk";
    case WIFI_AUTH_WPA2_PSK: return "wpa2-psk";
    case WIFI_AUTH_WPA_WPA2_PSK: return "wpa/wpa2-psk";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "wpa2-enterprise";
    case WIFI_AUTH_WPA3_PSK: return "wpa3-psk";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "wpa2/wpa3-psk";
    case WIFI_AUTH_WAPI_PSK: return "wapi-psk";
    default: return "unknown";
    }
}

static esp_err_t read_request_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    size_t remaining = req->content_len;
    size_t offset = 0;

    if (remaining + 1 > buf_len) {
        return ESP_ERR_NO_MEM;
    }

    while (remaining > 0) {
        int received = httpd_req_recv(req, buf + offset, remaining);
        if (received <= 0) {
            return ESP_FAIL;
        }
        offset += (size_t)received;
        remaining -= (size_t)received;
    }

    buf[offset] = '\0';
    return ESP_OK;
}

static bool json_escape_string(const char *src, char *dst, size_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len == 0) {
        return false;
    }

    size_t out = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p != '\0'; p++) {
        const char *esc = NULL;
        char unicode[7] = {0};

        switch (*p) {
        case '\\': esc = "\\\\"; break;
        case '"': esc = "\\\""; break;
        case '\b': esc = "\\b"; break;
        case '\f': esc = "\\f"; break;
        case '\n': esc = "\\n"; break;
        case '\r': esc = "\\r"; break;
        case '\t': esc = "\\t"; break;
        default:
            if (*p < 0x20) {
                snprintf(unicode, sizeof(unicode), "\\u%04x", *p);
                esc = unicode;
            }
            break;
        }

        if (esc != NULL) {
            size_t esc_len = strlen(esc);
            if (out + esc_len >= dst_len) {
                return false;
            }
            memcpy(dst + out, esc, esc_len);
            out += esc_len;
        } else {
            if (out + 1 >= dst_len) {
                return false;
            }
            dst[out++] = (char)*p;
        }
    }

    if (out >= dst_len) {
        return false;
    }

    dst[out] = '\0';
    return true;
}

static bool json_appendf(char *buf, size_t buf_len, size_t *off, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(buf + *off, buf_len - *off, fmt, ap);
    va_end(ap);

    if (written < 0 || (size_t)written >= (buf_len - *off)) {
        return false;
    }

    *off += (size_t)written;
    return true;
}

static esp_err_t json_status_handler(httpd_req_t *req)
{
    wifi_status_t wifi = {0};
    exlap_status_t exlap = {0};
    telemetry_t telemetry = {0};
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip = {0};
    esp_netif_ip_info_t ap_ip = {0};
    bool have_ip = false;
    bool have_ap_ip = false;
    char wifi_ssid_json[WIFI_MAX_SSID_LEN * 6 + 1] = {0};
    char wifi_ip_json[INET_ADDRSTRLEN * 2] = {0};
    char ap_ip_json[INET_ADDRSTRLEN * 2] = {0};

    wifi_manager_get_status(&wifi);
    exlap_client_get_status(&exlap);
    telemetry_db_get_snapshot(&telemetry);

    if (netif != NULL && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        have_ip = true;
    }
    if (ap_netif != NULL && esp_netif_get_ip_info(ap_netif, &ap_ip) == ESP_OK) {
        have_ap_ip = true;
    }

    if (!json_escape_string(wifi.ssid, wifi_ssid_json, sizeof(wifi_ssid_json))) {
        return ESP_ERR_NO_MEM;
    }

    char wifi_ip_fragment[128] = {0};
    char ap_ip_fragment[128] = {0};
    if (have_ip) {
        char ip_str[INET_ADDRSTRLEN] = {0};
        esp_ip4addr_ntoa(&ip.ip, ip_str, sizeof(ip_str));
        if (!json_escape_string(ip_str, wifi_ip_json, sizeof(wifi_ip_json))) {
            return ESP_ERR_NO_MEM;
        }
        snprintf(wifi_ip_fragment, sizeof(wifi_ip_fragment), ",\"ip\":\"%s\"", wifi_ip_json);
    }
    if (have_ap_ip) {
        char ap_ip_str[INET_ADDRSTRLEN] = {0};
        esp_ip4addr_ntoa(&ap_ip.ip, ap_ip_str, sizeof(ap_ip_str));
        if (!json_escape_string(ap_ip_str, ap_ip_json, sizeof(ap_ip_json))) {
            return ESP_ERR_NO_MEM;
        }
        snprintf(ap_ip_fragment, sizeof(ap_ip_fragment), ",\"ap_ip\":\"%s\"", ap_ip_json);
    }

    char body[4096];
    int written = snprintf(body, sizeof(body),
        "{"
        "\"uptime_ms\":%lu,"
        "\"wifi\":{\"state\":\"%s\",\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,\"auth_mode\":\"%s\"%s%s},"
        "\"ble\":{\"connected\":%s,\"conn_handle\":%u,\"mtu\":%u},"
        "\"can\":{\"running\":%s},"
        "\"exlap\":{\"state\":\"%s\",\"authenticated\":%s,\"packets_received\":%lu,\"packets_dropped\":%lu,\"bytes_received\":%lu,\"last_packet_ms\":%lu,\"last_auth_ms\":%lu,\"last_heartbeat_ms\":%lu},"
        "\"telemetry\":{\"timestamp_ms\":%lu,\"vehicle_speed_kmh\":%.1f,\"engine_rpm\":%.0f,\"throttle_pct\":%.1f,\"brake_pressure_bar\":%.2f,\"steering_angle_deg\":%.1f,\"yaw_rate\":%.2f,\"gear\":%d,\"wheel_speed_fl\":%.1f,\"wheel_speed_fr\":%.1f,\"wheel_speed_rl\":%.1f,\"wheel_speed_rr\":%.1f,\"avg_temp_lf\":%.1f,\"avg_temp_rf\":%.1f,\"avg_temp_lr\":%.1f,\"avg_temp_rr\":%.1f,\"max_temp\":%.1f,\"delta_temp_lf\":%.1f,\"delta_temp_rf\":%.1f,\"delta_temp_lr\":%.1f,\"delta_temp_rr\":%.1f,\"tirex_update_count\":%lu,\"exlap_update_count\":%lu,\"last_tirex_ms\":%lu,\"last_exlap_ms\":%lu,\"has_tirex_data\":%s,\"has_exlap_data\":%s},"
        "\"diagnostics\":{\"telemetry_rate_hz\":%.2f}"
        "}",
        (unsigned long)(esp_timer_get_time() / 1000),
        wifi_state_to_str(wifi.state),
        wifi_ssid_json,
        wifi.rssi,
        wifi.channel,
        auth_mode_to_str(wifi.auth_mode),
        wifi_ip_fragment,
        ap_ip_fragment,
        ble_is_connected() ? "true" : "false",
        ble_get_conn_handle(),
        ble_get_mtu(),
        can_driver_is_running() ? "true" : "false",
        exlap_state_to_str(exlap.state),
        exlap.authenticated ? "true" : "false",
        (unsigned long)exlap.packets_received,
        (unsigned long)exlap.packets_dropped,
        (unsigned long)exlap.bytes_received,
        (unsigned long)exlap.last_packet_ms,
        (unsigned long)exlap.last_auth_ms,
        (unsigned long)exlap.last_heartbeat_ms,
        (unsigned long)telemetry.timestamp_ms,
        telemetry.vehicle_speed_kmh,
        telemetry.engine_rpm,
        telemetry.throttle_pct,
        telemetry.brake_pressure_bar,
        telemetry.steering_angle_deg,
        telemetry.yaw_rate,
        telemetry.gear,
        telemetry.wheel_speed_fl,
        telemetry.wheel_speed_fr,
        telemetry.wheel_speed_rl,
        telemetry.wheel_speed_rr,
        telemetry.avg_temp_lf,
        telemetry.avg_temp_rf,
        telemetry.avg_temp_lr,
        telemetry.avg_temp_rr,
        telemetry.max_temp,
        telemetry.delta_temp_lf,
        telemetry.delta_temp_rf,
        telemetry.delta_temp_lr,
        telemetry.delta_temp_rr,
        (unsigned long)telemetry.tirex_update_count,
        (unsigned long)telemetry.exlap_update_count,
        (unsigned long)telemetry.last_tirex_ms,
        (unsigned long)telemetry.last_exlap_ms,
        telemetry.has_tirex_data ? "true" : "false",
        telemetry.has_exlap_data ? "true" : "false",
        telemetry_db_get_update_rate());

    if (written < 0 || (size_t)written >= sizeof(body)) {
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t json_config_handler(httpd_req_t *req)
{
    char wifi_ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char wifi_password[WIFI_MAX_PASS_LEN + 1] = {0};
    exlap_config_t config = {0};
    char wifi_ssid_json[WIFI_MAX_SSID_LEN * 6 + 1] = {0};
    char wifi_password_json[WIFI_MAX_PASS_LEN * 6 + 1] = {0};
    char vehicle_ip_json[EXLAP_MAX_IP_LEN * 6 + 1] = {0};
    char username_json[EXLAP_MAX_USER_LEN * 6 + 1] = {0};
    char password_json[EXLAP_MAX_PASS_LEN * 6 + 1] = {0};

    wifi_manager_load_credentials(wifi_ssid, wifi_password);
    exlap_client_load_config(&config);

    if (!json_escape_string(wifi_ssid, wifi_ssid_json, sizeof(wifi_ssid_json)) ||
        !json_escape_string(wifi_password, wifi_password_json, sizeof(wifi_password_json)) ||
        !json_escape_string(config.vehicle_ip, vehicle_ip_json, sizeof(vehicle_ip_json)) ||
        !json_escape_string(config.username, username_json, sizeof(username_json)) ||
        !json_escape_string(config.password, password_json, sizeof(password_json))) {
        return ESP_ERR_NO_MEM;
    }

    char body[768];
    int written = snprintf(body, sizeof(body),
        "{"
        "\"wifi_ssid\":\"%s\","
        "\"wifi_password\":\"%s\","
        "\"vehicle_ip\":\"%s\","
        "\"port\":%u,"
        "\"enabled\":%s,"
        "\"username\":\"%s\","
        "\"password\":\"%s\""
        "}",
        wifi_ssid_json,
        wifi_password_json,
        vehicle_ip_json,
        config.port,
        config.enabled ? "true" : "false",
        username_json,
        password_json);

    if (written < 0 || (size_t)written >= sizeof(body)) {
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t json_tirex_handler(httpd_req_t *req)
{
    tirex_snapshot_t *snapshot = calloc(1, sizeof(*snapshot));
    if (snapshot == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "tirex snapshot alloc failed");
        return ESP_FAIL;
    }
    if (!tirex_config_get_snapshot(snapshot)) {
        free(snapshot);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "tirex snapshot failed");
        return ESP_FAIL;
    }

    char *body = malloc(32768);
    if (body == NULL) {
        free(snapshot);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "tirex buffer alloc failed");
        return ESP_FAIL;
    }
    size_t off = 0;
    bool ok = true;

    if (!json_appendf(body, 32768, &off, "{\"sensor_count\":%u,\"sensors\":[",
                      snapshot->sensor_count)) {
        ok = false;
    }

    for (uint8_t i = 0; ok && i < snapshot->sensor_count; i++) {
        const tirex_sensor_snapshot_t *s = &snapshot->sensors[i];
        char name_json[TIREX_NAME_LEN * 6 + 1] = {0};
        if (!json_escape_string(s->name, name_json, sizeof(name_json))) {
            ok = false;
            break;
        }

        if (!json_appendf(body, 32768, &off,
                          "%s{\"present\":%s,\"base_id\":%lu,\"name\":\"%s\",\"corner\":%u,\"flip_orientation\":%s,"
                          "\"full_frame_mode\":%s,\"full_frame_trace_enabled\":%s,\"zone_count\":%u,\"sample_rate_code\":%u,"
                          "\"awaiting_config_confirm\":%s,\"requested_position\":%u,\"requested_zone_count\":%u,"
                          "\"requested_full_frame_mode\":%s,\"requested_sample_rate_code\":%u,"
                          "\"observed_position\":%u,\"observed_zone_count\":%u,\"observed_full_frame_mode\":%s,"
                          "\"last_seen_ms\":%lu,\"last_announce_ms\":%lu,\"last_stats_ms\":%lu,"
                          "\"last_config_ms\":%lu,\"last_apply_ms\":%lu,\"has_zone_data\":%s,"
                          "\"last_full_frame_ms\":%lu,\"full_frame_packet_count\":%lu,"
                          "\"full_frame_complete_count\":%lu,\"full_frame_discarded_count\":%lu,"
                          "\"full_frame_packets_collected\":%u,\"full_frame_sync_armed\":%s,"
                          "\"full_frame_synced\":%s,"
                          "\"has_full_frame_data\":%s,\"zones\":[",
                          (i == 0) ? "" : ",",
                          s->present ? "true" : "false",
                          (unsigned long)s->base_id,
                          name_json,
                          (unsigned)s->corner,
                          s->flip_orientation ? "true" : "false",
                          s->full_frame_mode ? "true" : "false",
                          s->full_frame_trace_enabled ? "true" : "false",
                          (unsigned)s->zone_count,
                          (unsigned)s->sample_rate_code,
                          s->awaiting_config_confirm ? "true" : "false",
                          (unsigned)s->requested_position,
                          (unsigned)s->requested_zone_count,
                          s->requested_full_frame_mode ? "true" : "false",
                          (unsigned)s->requested_sample_rate_code,
                          (unsigned)s->observed_position,
                          (unsigned)s->observed_zone_count,
                          s->observed_full_frame_mode ? "true" : "false",
                          (unsigned long)s->last_seen_ms,
                          (unsigned long)s->last_announce_ms,
                          (unsigned long)s->last_stats_ms,
                          (unsigned long)s->last_config_ms,
                          (unsigned long)s->last_apply_ms,
                          s->has_zone_data ? "true" : "false",
                          (unsigned long)s->last_full_frame_ms,
                          (unsigned long)s->full_frame_packet_count,
                          (unsigned long)s->full_frame_complete_count,
                          (unsigned long)s->full_frame_discarded_count,
                          (unsigned)s->full_frame_packets_collected,
                          s->full_frame_sync_armed ? "true" : "false",
                          s->full_frame_synced ? "true" : "false",
                          s->has_full_frame_data ? "true" : "false")) {
            ok = false;
            break;
        }

        for (uint8_t z = 0; z < TIREX_MAX_ZONES; z++) {
            if (!json_appendf(body, 32768, &off, "%s%.1f",
                              (z == 0) ? "" : ",",
                              s->zone_temps[z])) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            break;
        }

        if (!json_appendf(body, 32768, &off, "],\"frame_pixels\":[")) {
            ok = false;
            break;
        }

        for (uint16_t px = 0; px < TIREX_FULL_FRAME_PIXELS; px++) {
            if (!json_appendf(body, 32768, &off, "%s%.1f",
                              (px == 0) ? "" : ",",
                              s->full_frame_pixels[px])) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            break;
        }

        if (!json_appendf(body, 32768, &off, "],\"channel_pixels\":[")) {
            ok = false;
            break;
        }

        for (uint16_t channel = 0; channel < TIREX_FULL_FRAME_CHANNELS; channel++) {
            if (!json_appendf(body, 32768, &off, "%s%.1f",
                              (channel == 0) ? "" : ",",
                              s->full_frame_channels[channel])) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            break;
        }

        if (!json_appendf(body, 32768, &off, "]}")) {
            ok = false;
            break;
        }
    }

    if (ok && !json_appendf(body, 32768, &off, "]}")) {
        ok = false;
    }

    if (!ok) {
        free(snapshot);
        free(body);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "tirex json too large");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(snapshot);
    free(body);
    return ret;
}

static void write_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static esp_err_t tirex_live_frame_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char value[32] = {0};
    uint8_t channels[TIREX_FULL_FRAME_CHANNELS] = {0};
    uint8_t response[12 + TIREX_FULL_FRAME_CHANNELS] = {0};
    uint32_t frame_count = 0;
    uint32_t last_frame_ms = 0;
    bool synced = false;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "base_id", value, sizeof(value)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing base_id");
        return ESP_FAIL;
    }

    uint32_t base_id = (uint32_t)strtoul(value, NULL, 10);
    if (!tirex_config_get_live_frame(base_id, channels, &frame_count, &last_frame_ms, &synced)) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no complete TireX frame");
        return ESP_FAIL;
    }

    write_u32_le(&response[0], frame_count);
    write_u32_le(&response[4], last_frame_ms);
    response[8] = synced ? 1u : 0u;
    response[9] = 8u;
    response[10] = 12u;
    response[11] = 0u;
    memcpy(&response[12], channels, sizeof(channels));

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)response, sizeof(response));
}

static esp_err_t save_config_handler(httpd_req_t *req)
{
    char body[384];
    char wifi_ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char wifi_password[WIFI_MAX_PASS_LEN + 1] = {0};
    exlap_config_t config = {0};
    exlap_config_t current_config = {0};

    if (read_request_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }

    /* Start from the stored EXLAP config so curl can submit partial updates
     * without resetting fields that were not included in the request. */
    exlap_client_load_config(&current_config);
    config = current_config;

    if (httpd_query_key_value(body, "wifi_ssid", wifi_ssid, sizeof(wifi_ssid)) != ESP_OK) {
        wifi_ssid[0] = '\0';
    }
    if (httpd_query_key_value(body, "wifi_password", wifi_password, sizeof(wifi_password)) != ESP_OK) {
        wifi_password[0] = '\0';
    }
    if (httpd_query_key_value(body, "vehicle_ip", config.vehicle_ip, sizeof(config.vehicle_ip)) != ESP_OK) {
        strncpy(config.vehicle_ip, current_config.vehicle_ip, sizeof(config.vehicle_ip) - 1);
        config.vehicle_ip[sizeof(config.vehicle_ip) - 1] = '\0';
    }

    char value[128];
    if (httpd_query_key_value(body, "port", value, sizeof(value)) == ESP_OK) {
        config.port = (uint16_t)strtoul(value, NULL, 10);
    }
    if (httpd_query_key_value(body, "enabled", value, sizeof(value)) == ESP_OK) {
        config.enabled = (atoi(value) != 0);
    }
    if (httpd_query_key_value(body, "username", config.username, sizeof(config.username)) != ESP_OK) {
        strncpy(config.username, current_config.username, sizeof(config.username) - 1);
        config.username[sizeof(config.username) - 1] = '\0';
    }
    if (httpd_query_key_value(body, "password", config.password, sizeof(config.password)) != ESP_OK) {
        strncpy(config.password, current_config.password, sizeof(config.password) - 1);
        config.password[sizeof(config.password) - 1] = '\0';
    }

    if (strlen(wifi_ssid) > 0) {
        if (!wifi_manager_save_credentials(wifi_ssid, wifi_password)) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "wifi save failed");
            return ESP_FAIL;
        }
    }

    if (!exlap_client_save_config(&config)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
        return ESP_FAIL;
    }

    if (config.enabled && wifi_manager_is_connected()) {
        exlap_client_disconnect();
        if (strlen(config.vehicle_ip) > 0) {
            exlap_client_connect(config.vehicle_ip, config.port);
        }
    } else if (!config.enabled) {
        exlap_client_disconnect();
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t tirex_config_handler(httpd_req_t *req)
{
    char body[512];
    char value[64];
    uint32_t base_id;
    tirex_sensor_profile_t profile = {0};
    bool apply = false;
    bool applied = false;

    if (read_request_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }

    if (httpd_query_key_value(body, "base_id", value, sizeof(value)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing base_id");
        return ESP_FAIL;
    }
    base_id = (uint32_t)strtoul(value, NULL, 10);

    if (!tirex_config_get_profile(base_id, &profile)) {
        memset(&profile, 0, sizeof(profile));
        profile.zone_count = 4;
        profile.sample_rate_code = 16;
    }

    if (httpd_query_key_value(body, "name", profile.name, sizeof(profile.name)) != ESP_OK) {
        profile.name[0] = '\0';
    }

    if (httpd_query_key_value(body, "corner", value, sizeof(value)) == ESP_OK) {
        profile.corner = (tirex_corner_t)strtoul(value, NULL, 10);
    }
    if (httpd_query_key_value(body, "flip_orientation", value, sizeof(value)) == ESP_OK) {
        profile.flip_orientation = (atoi(value) != 0);
    } else {
        profile.flip_orientation = false;
    }
    if (httpd_query_key_value(body, "full_frame_mode", value, sizeof(value)) == ESP_OK) {
        profile.full_frame_mode = (atoi(value) != 0);
    } else {
        profile.full_frame_mode = false;
    }
    if (httpd_query_key_value(body, "full_frame_trace_enabled", value, sizeof(value)) == ESP_OK) {
        profile.full_frame_trace_enabled = (atoi(value) != 0);
    } else {
        profile.full_frame_trace_enabled = false;
    }
    if (httpd_query_key_value(body, "zone_count", value, sizeof(value)) == ESP_OK) {
        profile.zone_count = (uint8_t)strtoul(value, NULL, 10);
    }
    if (httpd_query_key_value(body, "sample_rate_code", value, sizeof(value)) == ESP_OK) {
        profile.sample_rate_code = (uint8_t)strtoul(value, NULL, 10);
    }
    if (httpd_query_key_value(body, "apply", value, sizeof(value)) == ESP_OK) {
        apply = (atoi(value) != 0);
    }

    if (!tirex_config_update_profile(base_id, &profile)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
        return ESP_FAIL;
    }

    if (apply) {
        applied = tirex_config_apply(base_id);
    }

    char response[96];
    snprintf(response, sizeof(response),
             "{\"ok\":true,\"applied\":%s}", applied ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, response);
}

static esp_err_t wifi_connect_handler(httpd_req_t *req)
{
    char wifi_ssid[WIFI_MAX_SSID_LEN + 1] = {0};
    char wifi_password[WIFI_MAX_PASS_LEN + 1] = {0};

    if (!wifi_manager_load_credentials(wifi_ssid, wifi_password)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no saved wifi credentials");
        return ESP_FAIL;
    }

    if (!wifi_manager_connect(wifi_ssid, wifi_password)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "wifi connect failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t wifi_disconnect_handler(httpd_req_t *req)
{
    wifi_manager_disconnect();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

bool web_ui_init(void)
{
    if (s_httpd != NULL) {
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 10;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        s_httpd = NULL;
        return false;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
    };
    const httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = json_status_handler,
    };
    const httpd_uri_t config_uri = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = json_config_handler,
    };
    const httpd_uri_t config_post_uri = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = save_config_handler,
    };
    const httpd_uri_t tirex_uri = {
        .uri = "/api/tirex",
        .method = HTTP_GET,
        .handler = json_tirex_handler,
    };
    const httpd_uri_t tirex_post_uri = {
        .uri = "/api/tirex/config",
        .method = HTTP_POST,
        .handler = tirex_config_handler,
    };
    const httpd_uri_t tirex_live_uri = {
        .uri = "/api/tirex/frame",
        .method = HTTP_GET,
        .handler = tirex_live_frame_handler,
    };
    const httpd_uri_t wifi_connect_uri = {
        .uri = "/api/wifi/connect",
        .method = HTTP_POST,
        .handler = wifi_connect_handler,
    };
    const httpd_uri_t wifi_disconnect_uri = {
        .uri = "/api/wifi/disconnect",
        .method = HTTP_POST,
        .handler = wifi_disconnect_handler,
    };

    httpd_register_uri_handler(s_httpd, &index_uri);
    httpd_register_uri_handler(s_httpd, &status_uri);
    httpd_register_uri_handler(s_httpd, &config_uri);
    httpd_register_uri_handler(s_httpd, &config_post_uri);
    httpd_register_uri_handler(s_httpd, &tirex_uri);
    httpd_register_uri_handler(s_httpd, &tirex_post_uri);
    httpd_register_uri_handler(s_httpd, &tirex_live_uri);
    httpd_register_uri_handler(s_httpd, &wifi_connect_uri);
    httpd_register_uri_handler(s_httpd, &wifi_disconnect_uri);

    ESP_LOGI(TAG, "Web UI started");
    return true;
}

void web_ui_stop(void)
{
    if (s_httpd != NULL) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
        ESP_LOGI(TAG, "Web UI stopped");
    }
}
#else

bool web_ui_init(void)
{
    ESP_LOGI(TAG, "Web UI disabled by build flag");
    return true;
}

void web_ui_stop(void)
{
    /* No-op when the troubleshooting UI is disabled. */
}

#endif /* CONFIG_EXLAP_TESTING_AP_UI */
