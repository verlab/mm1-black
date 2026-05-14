/* web_portal.cpp — ESP32 SoftAP + dark MIRA dashboard.
 *
 *  - Runs an open or WPA2 access point so the surveyor's phone can browse,
 *    inspect and download survey artefacts (CSV files on SD, live point
 *    table) over plain HTTP — no app install, no pairing.
 *  - Single-file inline UI (dark, "tech" aesthetic, English copy).
 *  - The on-board MIRA "splash" mark is rendered as inline SVG.
 *  - Soft-AP is manual from SETUP Wi-Fi (no RF at boot) for stable power-on.
 */

#include "web_portal.h"

#ifdef ARDUINO_ARCH_ESP32

#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>

namespace web_portal {

namespace {

constexpr uint16_t kHttpPort   = 80;
constexpr uint8_t  kApChannel  = 6; /* common home channel; ch.1 was marginal on some phones */
constexpr uint8_t  kMaxClients = 4;

WebServer  g_server(kHttpPort);
Callbacks  g_cb{};
bool       g_running = false;
char       g_ip[20]  = "0.0.0.0";

/* ---------- inline page ---------------------------------------------------- */

constexpr char kDashboardHtml[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
<title>MIRA · MM1 Console</title>
<style>
:root{
  --bg:#0b0f17; --panel:#121826; --panel-2:#0e1422; --border:#1d2740;
  --text:#e6edf7; --muted:#8b97ad; --accent:#5bc0ff; --accent-2:#7c5cff;
  --ok:#36d399; --warn:#f5b342; --err:#ff5e7e;
  --mono:ui-monospace,SFMono-Regular,Consolas,"JetBrains Mono",monospace;
}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  -webkit-font-smoothing:antialiased;line-height:1.45}
a{color:var(--accent);text-decoration:none}
a:hover{color:#90d6ff}
.container{max-width:1100px;margin:0 auto;padding:24px 18px 80px}
.bg-grid{position:fixed;inset:0;z-index:-1;
  background:
    radial-gradient(1200px 600px at 80% -10%,rgba(124,92,255,.15),transparent 60%),
    radial-gradient(900px 500px at -10% 110%,rgba(91,192,255,.12),transparent 60%),
    linear-gradient(180deg,#0b0f17 0%,#070a12 100%);}
header{display:flex;align-items:center;gap:14px;margin-bottom:22px;padding-top:6px}
.logo{display:flex;align-items:center;gap:12px}
.logo svg{height:38px;width:auto}
.brand{font-weight:700;font-size:22px;letter-spacing:.18em}
.brand small{display:block;font-weight:500;color:var(--muted);font-size:11px;
  letter-spacing:.32em;margin-top:2px}
.pill{display:inline-flex;align-items:center;gap:6px;
  padding:4px 10px;border-radius:999px;font-size:12px;font-family:var(--mono);
  background:#0f1626;border:1px solid var(--border);color:var(--muted)}
.dot{width:8px;height:8px;border-radius:50%}
.ok .dot{background:var(--ok);box-shadow:0 0 0 3px rgba(54,211,153,.18)}
.warn .dot{background:var(--warn)}
.err .dot{background:var(--err)}
.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:14px}
.card{background:linear-gradient(180deg,var(--panel) 0%,var(--panel-2) 100%);
  border:1px solid var(--border);border-radius:14px;padding:16px 16px 14px;
  box-shadow:0 14px 40px -22px rgba(0,0,0,.6)}
.card h3{margin:0 0 10px;font-size:13px;font-weight:600;letter-spacing:.16em;
  text-transform:uppercase;color:var(--muted)}
.col-4{grid-column:span 4}.col-6{grid-column:span 6}.col-8{grid-column:span 8}
.col-12{grid-column:span 12}
@media (max-width:780px){.col-4,.col-6,.col-8{grid-column:span 12}}
.kv{display:grid;grid-template-columns:max-content 1fr;gap:6px 14px;font-family:var(--mono);font-size:13px}
.kv b{color:var(--muted);font-weight:500}
.kv span{color:var(--text);word-break:break-all}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:8px 10px;text-align:left;border-bottom:1px solid var(--border)}
th{font-size:11px;letter-spacing:.16em;text-transform:uppercase;color:var(--muted);font-weight:600}
tr:hover td{background:rgba(91,192,255,.06)}
.btn{display:inline-flex;align-items:center;gap:6px;padding:7px 12px;
  border-radius:8px;border:1px solid var(--border);background:#172033;
  color:var(--text);font-family:var(--mono);font-size:12px;cursor:pointer;
  text-decoration:none}
.btn:hover{border-color:var(--accent);color:var(--accent)}
.btn.primary{background:linear-gradient(180deg,#1f3a66,#15264a);border-color:#2f558c;color:#cfe6ff}
.footer{margin-top:30px;color:var(--muted);font-size:12px;text-align:center}
.spark{margin-top:4px;font-family:var(--mono);font-size:11px;color:var(--muted)}
.badge{font-family:var(--mono);font-size:11px;padding:2px 6px;border-radius:4px;
  background:#0f1626;border:1px solid var(--border);color:var(--muted)}
.empty{padding:24px;text-align:center;color:var(--muted);font-style:italic}
</style></head><body>
<div class="bg-grid"></div>
<div class="container">
  <header>
    <div class="logo">
      <svg viewBox="0 0 240 60" xmlns="http://www.w3.org/2000/svg">
        <defs>
          <linearGradient id="g1" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stop-color="#5bc0ff"/>
            <stop offset="100%" stop-color="#7c5cff"/>
          </linearGradient>
        </defs>
        <g fill="none" stroke="url(#g1)" stroke-width="3" stroke-linecap="round" stroke-linejoin="round">
          <path d="M8 50 L24 14 L40 38 L56 14 L72 50"/>
          <path d="M88 50 L88 14 M88 14 L116 50 L116 14"/>
          <circle cx="148" cy="32" r="14"/>
          <path d="M176 14 L176 50 M176 14 L200 14 Q210 14 210 24 Q210 34 200 34 L182 34 L210 50"/>
        </g>
      </svg>
      <div class="brand">MIRA<small>MM1 · Console</small></div>
    </div>
    <span id="state" class="pill ok"><span class="dot"></span><span id="state-txt">online</span></span>
  </header>

  <div class="grid">
    <section class="card col-8">
      <h3>Device</h3>
      <div class="kv">
        <b>Firmware</b><span id="fw">—</span>
        <b>Device name</b><span id="dname">—</span>
        <b>Active CSV</b><span id="csv">—</span>
        <b>Stored points</b><span id="pts">—</span>
        <b>Bluetooth</b><span id="bt">—</span>
        <b>BT MAC</b><span id="btmac">—</span>
        <b>Paired phone</b><span id="btpeer">—</span>
        <b>AP IP</b><span id="apip">—</span>
        <b>AP SSID / pass</b><span id="apssid">—</span>
        <b>AP clients</b><span id="apcli">—</span>
      </div>
      <div class="spark">Live snapshot from /api/status · refreshes every 3 s.</div>
    </section>

    <section class="card col-4">
      <h3>Quick actions</h3>
      <div style="display:flex;flex-direction:column;gap:8px">
        <a class="btn primary" href="/points.csv" download>Download current points (CSV)</a>
        <a class="btn" href="/api/files">Files JSON</a>
        <a class="btn" href="/api/status">Status JSON</a>
        <a class="btn" href="/api/points">Points JSON</a>
      </div>
      <div class="spark" style="margin-top:12px">All endpoints are read-only.</div>
    </section>

    <section class="card col-12">
      <h3>SD card · CSV files</h3>
      <div id="files-wrap"><div class="empty">Loading…</div></div>
    </section>

    <section class="card col-12">
      <h3>Points table (RAM)</h3>
      <div id="points-wrap"><div class="empty">Loading…</div></div>
    </section>
  </div>

  <div class="footer">MIRA · MM1-BLACK · cave survey console · served from ESP32</div>
</div>
<script>
const $=q=>document.querySelector(q);
function fmtBytes(n){if(n<1024)return n+' B';if(n<1048576)return (n/1024).toFixed(1)+' KiB';return (n/1048576).toFixed(2)+' MiB';}
async function refreshStatus(){
  try{
    const r=await fetch('/api/status'); if(!r.ok) throw 0; const j=await r.json();
    $('#fw').textContent=j.fw;
    $('#dname').textContent=j.dev;
    $('#csv').textContent=j.csv;
    $('#pts').textContent=j.pts;
    $('#bt').innerHTML = j.link
      ? '<span class="badge" style="color:var(--ok);border-color:var(--ok)">LINKED</span> active SPP session'
      : (j.paired
        ? '<span class="badge" style="color:var(--accent);border-color:var(--accent)">BONDED</span> awaiting TopoDroid'
        : '<span class="badge" style="color:var(--muted)">IDLE</span> not paired');
    $('#btmac').textContent=j.btmac||'—';
    $('#btpeer').textContent=j.peer||'—';
    $('#apip').textContent=j.apip;
    $('#apssid').textContent=j.ssid+' / '+(j.pass||'(open)');
    $('#apcli').textContent=j.cli;
    $('#state-txt').textContent='online · '+j.up+' s';
  }catch(e){ $('#state-txt').textContent='offline'; $('#state').className='pill err'; }
}
async function refreshFiles(){
  try{
    const r=await fetch('/api/files'); const j=await r.json();
    if(!j.files || !j.files.length){ $('#files-wrap').innerHTML='<div class="empty">No CSV files on SD.</div>'; return; }
    let h='<table><thead><tr><th>Name</th><th>Size</th><th></th></tr></thead><tbody>';
    for(const f of j.files){
      h+=`<tr><td>${f.name}</td><td>${fmtBytes(f.size)}</td><td style="text-align:right"><a class="btn" href="/sd/${encodeURIComponent(f.name)}" download>Download</a></td></tr>`;
    }
    h+='</tbody></table>';
    $('#files-wrap').innerHTML=h;
  }catch(e){ $('#files-wrap').innerHTML='<div class="empty">Failed to load file list.</div>'; }
}
async function refreshPoints(){
  try{
    const r=await fetch('/api/points'); const j=await r.json();
    if(!j.points || !j.points.length){ $('#points-wrap').innerHTML='<div class="empty">No points captured yet.</div>'; return; }
    let h='<table><thead><tr><th>#</th><th>ID</th><th>Type</th><th>Dist (m)</th><th>Azimuth</th><th>Incl.</th><th>Roll</th><th>Temp</th><th>Time</th></tr></thead><tbody>';
    j.points.forEach((p,i)=>{
      h+=`<tr><td>${i+1}</td><td>${p.id}</td><td>${p.type}</td><td>${(+p.dist).toFixed(3)}</td><td>${(+p.az).toFixed(2)}</td><td>${(+p.inc).toFixed(2)}</td><td>${(+p.roll).toFixed(2)}</td><td>${(+p.temp).toFixed(1)}</td><td>${p.ts||''}</td></tr>`;
    });
    h+='</tbody></table>';
    $('#points-wrap').innerHTML=h;
  }catch(e){ $('#points-wrap').innerHTML='<div class="empty">Failed to load points.</div>'; }
}
refreshStatus();refreshFiles();refreshPoints();
setInterval(refreshStatus,3000);
setInterval(refreshFiles,8000);
setInterval(refreshPoints,4000);
</script>
</body></html>
)HTML";

/* ---------- helpers -------------------------------------------------------- */

void send_status_json()
{
    Status st{};
    if (g_cb.get_status) g_cb.get_status(st);
    String j = "{";
    j += "\"fw\":\"";   j += st.fw_version ? st.fw_version : ""; j += "\",";
    j += "\"dev\":\"";  j += st.device_name ? st.device_name : ""; j += "\",";
    j += "\"csv\":\"";  j += st.active_csv ? st.active_csv : ""; j += "\",";
    j += "\"pts\":";    j += (uint32_t)st.point_count; j += ",";
    j += "\"link\":";   j += st.bt_linked ? "true" : "false"; j += ",";
    j += "\"paired\":"; j += st.bt_paired ? "true" : "false"; j += ",";
    j += "\"btmac\":\""; j += st.bt_local_mac ? st.bt_local_mac : ""; j += "\",";
    j += "\"peer\":\""; j += st.bt_peer_mac ? st.bt_peer_mac : ""; j += "\",";
    j += "\"apip\":\""; j += st.ap_ip ? st.ap_ip : ""; j += "\",";
    j += "\"ssid\":\""; j += st.ap_ssid ? st.ap_ssid : ""; j += "\",";
    j += "\"pass\":\""; j += st.ap_password ? st.ap_password : ""; j += "\",";
    j += "\"cli\":";    j += (uint32_t)st.wifi_clients; j += ",";
    j += "\"up\":";     j += (uint32_t)(millis() / 1000);
    j += "}";
    g_server.send(200, "application/json", j);
}

void send_files_json()
{
    String j = "{\"files\":[";
    File root = SD.open("/");
    bool first = true;
    if (root && root.isDirectory()) {
        File e;
        while ((e = root.openNextFile())) {
            if (!e.isDirectory()) {
                String n = e.name();
                if (n.endsWith(".csv") || n.endsWith(".CSV")) {
                    if (!first) j += ",";
                    j += "{\"name\":\"";
                    /* trim leading slash if SD library prefixes */
                    if (n.length() > 0 && n[0] == '/') n.remove(0, 1);
                    j += n;
                    j += "\",\"size\":";
                    j += (uint32_t)e.size();
                    j += "}";
                    first = false;
                }
            }
            e.close();
        }
    }
    j += "]}";
    g_server.send(200, "application/json", j);
}

/* Sink that appends to a String — used to render in-RAM points into a JSON
 * payload via the host-supplied write_points_csv callback. */
struct StringSink {
    String s;
};

void sink_append(const char* p, size_t n, void* user)
{
    StringSink* ss = static_cast<StringSink*>(user);
    ss->s.concat(p, n);
}

void send_points_csv()
{
    if (!g_cb.write_points_csv) {
        g_server.send(503, "text/plain", "points unavailable");
        return;
    }
    StringSink ss;
    ss.s.reserve(8 * 1024);
    g_cb.write_points_csv(sink_append, &ss);
    g_server.sendHeader("Content-Disposition", "attachment; filename=mm1_points.csv");
    g_server.send(200, "text/csv", ss.s);
}

void send_points_json()
{
    if (!g_cb.write_points_json) {
        g_server.send(503, "application/json", "{\"points\":[]}");
        return;
    }
    StringSink ss;
    ss.s.reserve(8 * 1024);
    g_cb.write_points_json(sink_append, &ss);
    g_server.send(200, "application/json", ss.s);
}

void send_sd_file()
{
    String path = g_server.uri();
    /* /sd/<name>  →  /<name> */
    if (path.startsWith("/sd/")) path.remove(0, 3);
    if (path.length() == 0 || path == "/") { g_server.send(400, "text/plain", "bad path"); return; }
    if (path.indexOf("..") >= 0)            { g_server.send(403, "text/plain", "forbidden"); return; }

    File f = SD.open(path, FILE_READ);
    if (!f || f.isDirectory()) { g_server.send(404, "text/plain", "not found"); return; }
    g_server.sendHeader("Content-Disposition", String("attachment; filename=") + path.substring(path.lastIndexOf('/') + 1));
    g_server.streamFile(f, "text/csv");
    f.close();
}

void on_root() { g_server.send_P(200, "text/html", kDashboardHtml); }
void on_404()  { g_server.send(404, "text/plain", "not found"); }

}  // namespace

/* ---------- public API ----------------------------------------------------- */

bool start(const char* ssid, const char* password, const Callbacks& cb)
{
    if (g_running) return true;
    g_cb = cb;

    /* Minimal path: AP-only mode. AP_STA + coexist tweaks were linked to boot loops on CYD. */
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    delay(50);
    if (!WiFi.mode(WIFI_AP)) {
        g_cb = {};
        return false;
    }
    WiFi.setSleep(false);

    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));

    bool ok = false;
    if (password && password[0] && strlen(password) >= 8) {
        ok = WiFi.softAP(ssid, password, kApChannel, 0, kMaxClients);
    } else {
        ok = WiFi.softAP(ssid, nullptr, kApChannel, 0, kMaxClients);
    }

    if (!ok) {
        WiFi.softAPdisconnect(true);
        delay(40);
        WiFi.mode(WIFI_OFF);
        g_cb = {};
        return false;
    }

    IPAddress ip = WiFi.softAPIP();
    snprintf(g_ip, sizeof(g_ip), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

    g_server.on("/",            HTTP_GET, on_root);
    g_server.on("/index.html",  HTTP_GET, on_root);
    g_server.on("/api/status",  HTTP_GET, send_status_json);
    g_server.on("/api/files",   HTTP_GET, send_files_json);
    g_server.on("/api/points",  HTTP_GET, send_points_json);
    g_server.on("/points.csv",  HTTP_GET, send_points_csv);
    g_server.onNotFound([] {
        if (g_server.uri().startsWith("/sd/")) send_sd_file();
        else on_404();
    });
    g_server.begin();
    g_running = true;
    return true;
}

void stop()
{
    if (!g_running) return;
    g_server.close();
    delay(20);
    g_server.stop();
    WiFi.softAPdisconnect(true);
    delay(80);
    WiFi.mode(WIFI_OFF);
    g_running = false;
    g_cb = {};
    g_ip[0] = '0';
    g_ip[1] = '\0';
}

bool        running()   { return g_running; }
uint8_t     clients()   { return g_running ? WiFi.softAPgetStationNum() : 0; }
const char* ap_ip()     { return g_ip; }

void loop()             { if (g_running) g_server.handleClient(); }

}  // namespace web_portal

#endif  // ARDUINO_ARCH_ESP32
