/**
 * MM1-BLACK WebSerial firmware updater (issue #15).
 * Chrome/Edge + HTTPS. Firmware list from releases.json (private repo friendly).
 */

const REPO = "verlab/cyd_brics5_mm1";
const BIN_PREFIX = "MM1-BLACK-denky32-";
const FLASH_ADDR = 0x10000;
const VERSION_BAUD = 9600;
const FLASH_BAUD = 115200;

let selectedPort = null;
let releases = [];
let deviceVersion = null;
let esptoolModule = null;

const $ = (id) => document.getElementById(id);
const logEl = $("log");
const progressWrap = $("progressWrap");
const progressBar = $("progressBar");

function log(msg) {
  const t = new Date().toLocaleTimeString();
  logEl.textContent += `[${t}] ${msg}\n`;
  logEl.scrollTop = logEl.scrollHeight;
}

function clearLog() {
  logEl.textContent = "";
}

function setStatus(msg, kind = "") {
  const el = $("statusLine");
  if (!el) return;
  el.textContent = msg || "";
  el.className = kind ? `status-line ${kind}` : "status-line";
}

function setProgress(pct) {
  progressWrap.classList.add("visible");
  progressBar.style.width = `${Math.min(100, Math.max(0, pct))}%`;
}

function hideProgress() {
  progressWrap.classList.remove("visible");
  progressBar.style.width = "0%";
}

function parseTagVer(tag) {
  const m = /^v?(\d+)\.(\d+)\.(\d+)/i.exec(tag || "");
  if (!m) return null;
  return { major: +m[1], minor: +m[2], patch: +m[3], raw: tag };
}

function compareVer(a, b) {
  if (!a || !b) return 0;
  if (a.major !== b.major) return a.major - b.major;
  if (a.minor !== b.minor) return a.minor - b.minor;
  return a.patch - b.patch;
}

function normalizeDeviceVer(s) {
  if (!s) return null;
  const m = /v?(\d+\.\d+\.\d+)/i.exec(s);
  return m ? parseTagVer(m[1]) : parseTagVer(s);
}

function updateVersionUI() {
  const dev = $("deviceVersion");
  const sel = $("selectedVersion");
  const cmp = $("versionCompare");

  if (deviceVersion) {
    dev.textContent = deviceVersion;
    dev.parentElement.classList.add("ok");
    dev.parentElement.classList.remove("muted");
  } else {
    dev.textContent = "Not read";
    dev.parentElement.classList.add("muted");
    dev.parentElement.classList.remove("ok");
  }

  const rel = releases.find((r) => r.tag === $("releaseSelect").value);
  if (rel) {
    sel.textContent = rel.tag;
    const dv = normalizeDeviceVer(deviceVersion);
    const rv = parseTagVer(rel.tag);
    if (dv && rv) {
      const c = compareVer(rv, dv);
      if (c > 0)
        cmp.innerHTML =
          '<span class="compare-newer">A newer release is available.</span>';
      else if (c < 0)
        cmp.innerHTML =
          '<span class="compare-older">Device is newer than this release.</span>';
      else cmp.textContent = "Device matches this release.";
    } else {
      cmp.textContent = "";
    }
  } else {
    sel.textContent = "—";
    cmp.textContent = "";
  }

  $("btnFlash").disabled = !releases.length || !selectedPort || !$("ackFlash").checked;
}

function populateReleaseSelect() {
  const sel = $("releaseSelect");
  sel.innerHTML = "";

  if (!releases.length) {
    sel.innerHTML =
      '<option value="">No firmware images — check releases.json on Pages</option>';
    setStatus(
      "No releases in manifest. After a git tag release, bins are copied to this site.",
      "err"
    );
    updateVersionUI();
    return;
  }

  for (const r of releases) {
    const opt = document.createElement("option");
    opt.value = r.tag;
    const kb = r.size ? ` (${(r.size / 1024).toFixed(0)} KB)` : "";
    opt.textContent = `${r.tag}${kb}`;
    sel.appendChild(opt);
  }
  setStatus(`${releases.length} release(s) ready.`, "ok");
  updateVersionUI();
  log(`Loaded ${releases.length} release(s) from manifest.`);
}

async function fetchReleasesFromManifest() {
  const res = await fetch("./releases.json", { cache: "no-store" });
  if (!res.ok) throw new Error(`releases.json HTTP ${res.status}`);
  const data = await res.json();
  const out = [];
  for (const rel of data.releases || []) {
    if (!rel.tag || !rel.url) continue;
    out.push({
      tag: rel.tag,
      name: rel.name || rel.tag,
      url: new URL(rel.url, window.location.href).href,
      size: rel.size || 0,
    });
  }
  return out;
}

async function fetchReleasesFromGitHub() {
  const res = await fetch(`https://api.github.com/repos/${REPO}/releases`);
  if (res.status === 404)
    throw new Error(
      "GitHub API: repo private or missing — firmware list uses releases.json on Pages"
    );
  if (!res.ok) throw new Error(`GitHub API HTTP ${res.status}`);
  const data = await res.json();
  const out = [];
  for (const rel of data) {
    if (rel.draft || rel.prerelease) continue;
    const tag = rel.tag_name || rel.name;
    const asset = (rel.assets || []).find(
      (a) => a.name.startsWith(BIN_PREFIX) && a.name.endsWith(".bin")
    );
    if (!asset) continue;
    out.push({
      tag,
      name: rel.name || tag,
      url: asset.browser_download_url,
      size: asset.size,
    });
  }
  return out;
}

async function fetchReleases() {
  const sel = $("releaseSelect");
  sel.innerHTML = '<option value="">Loading…</option>';
  setStatus("Loading firmware list…");
  releases = [];

  try {
    releases = await fetchReleasesFromManifest();
  } catch (e) {
    log(`Manifest: ${e.message}`);
  }

  if (!releases.length) {
    try {
      releases = await fetchReleasesFromGitHub();
      log("Loaded releases from GitHub API.");
    } catch (e) {
      log(`GitHub API: ${e.message}`);
    }
  }

  populateReleaseSelect();
}

async function readVersionFromPort(port) {
  let reader;
  let writer;
  try {
    await port.open({ baudRate: VERSION_BAUD });
    await new Promise((r) => setTimeout(r, 300));
    writer = port.writable.getWriter();
    reader = port.readable.getReader();
    await writer.write(new TextEncoder().encode("VERSION\n"));
    await writer.releaseLock();
    writer = null;

    const dec = new TextDecoder();
    let buf = "";
    const deadline = Date.now() + 3000;
    while (Date.now() < deadline) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += dec.decode(value, { stream: true });
      const m = /MM1_FW_VERSION=([^\r\n]+)/.exec(buf);
      if (m) return m[1].trim();
    }
    return null;
  } finally {
    try {
      if (writer) await writer.releaseLock();
    } catch (_) {}
    try {
      if (reader) await reader.releaseLock();
    } catch (_) {}
    try {
      await port.close();
    } catch (_) {}
  }
}

async function connectAndReadVersion() {
  clearLog();
  hideProgress();
  setStatus("Select the USB serial port in the browser dialog…");

  if (!("serial" in navigator)) {
    log("Web Serial not supported. Use Chrome or Edge on a desktop PC.");
    setStatus("Web Serial unavailable in this browser.", "err");
    return;
  }

  try {
    selectedPort = await navigator.serial.requestPort();
    log("Port selected. Reading firmware version @ 9600 baud…");
    setStatus("Reading VERSION from device…");
    deviceVersion = await readVersionFromPort(selectedPort);
    if (deviceVersion) {
      log(`Device reports: ${deviceVersion}`);
      setStatus(`Connected — firmware ${deviceVersion}`, "ok");
    } else {
      log(
        "No VERSION line in 3 s (device busy or wrong port). You can still flash."
      );
      setStatus("USB connected — version not read (flash still OK).", "ok");
    }
    updateVersionUI();
  } catch (e) {
    if (e.name === "NotFoundError") {
      log("No port selected.");
      setStatus("No port selected.", "err");
    } else {
      log(`Error: ${e.message || e}`);
      setStatus(e.message || String(e), "err");
    }
    deviceVersion = null;
    selectedPort = null;
    updateVersionUI();
  }
}

async function loadEsptool() {
  if (!esptoolModule) {
    log("Loading esptool-js…");
    esptoolModule = await import(
      "https://unpkg.com/esptool-js@0.5.4/lib/index.js?module"
    );
  }
  return esptoolModule;
}

async function downloadFirmware(url) {
  log(`Downloading ${url}…`);
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Download failed: HTTP ${res.status}`);
  return new Uint8Array(await res.arrayBuffer());
}

async function flashFirmware() {
  const tag = $("releaseSelect").value;
  const rel = releases.find((r) => r.tag === tag);
  if (!rel) {
    log("Select a release.");
    return;
  }
  if (!selectedPort) {
    log("Connect USB first (Device → Connect).");
    return;
  }
  if (!$("ackFlash").checked) {
    log("Confirm the disclaimer checkbox.");
    return;
  }

  clearLog();
  setProgress(0);
  $("btnFlash").disabled = true;
  $("btnConnect").disabled = true;
  setStatus("Flashing… do not unplug USB.");

  const terminal = {
    clean: () => {},
    writeLine: (d) => log(d),
    write: (d) => log(d),
  };

  try {
    const firmware = await downloadFirmware(rel.url);
    log(`Downloaded ${firmware.byteLength} bytes (${rel.tag}).`);

    const { ESPLoader, Transport } = await loadEsptool();
    const transport = new Transport(selectedPort, true);
    const loader = new ESPLoader({
      transport,
      baudrate: FLASH_BAUD,
      terminal,
      debugLogging: false,
    });

    log("Connecting to ESP32 @ 115200…");
    const chip = await loader.main();
    log(`Chip: ${chip}`);

    log("Writing flash…");
    await loader.writeFlash({
      fileArray: [{ data: firmware, address: FLASH_ADDR }],
      flashMode: "dio",
      flashFreq: "40m",
      flashSize: "4MB",
      eraseAll: false,
      compress: true,
      reportProgress: (_idx, written, total) => {
        setProgress((written / total) * 100);
      },
    });

    log("Resetting device…");
    await loader.after("hard_reset");
    await transport.disconnect();

    log("Flash complete.");
    deviceVersion = rel.tag.replace(/^v/, "");
    setStatus(`Flashed ${rel.tag} successfully.`, "ok");
    updateVersionUI();
  } catch (e) {
    log(`Flash failed: ${e.message || e}`);
    setStatus(`Flash failed: ${e.message || e}`, "err");
  } finally {
    hideProgress();
    $("btnConnect").disabled = false;
    updateVersionUI();
  }
}

function init() {
  log("Ready. Use Chrome or Edge on desktop with USB connected.");
  if (!("serial" in navigator)) {
    log("Web Serial not supported in this browser.");
    $("btnConnect").disabled = true;
    setStatus("Use Chrome or Edge on desktop.", "err");
  }

  $("btnConnect").addEventListener("click", connectAndReadVersion);
  $("btnFlash").addEventListener("click", flashFirmware);
  $("releaseSelect").addEventListener("change", updateVersionUI);
  $("ackFlash").addEventListener("change", updateVersionUI);
  $("btnRefreshReleases").addEventListener("click", () => {
    fetchReleases().catch((e) => {
      log(`Releases: ${e.message}`);
      setStatus(e.message, "err");
    });
  });

  fetchReleases().catch((e) => {
    log(`Releases: ${e.message}`);
    setStatus(e.message, "err");
  });
}

init();
