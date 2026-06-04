/**
 * MM1-BLACK WebSerial firmware updater (issue #15).
 * Requires Chrome/Edge and HTTPS (GitHub Pages).
 */

import { ESPLoader, Transport } from "https://unpkg.com/esptool-js@0.5.4/lib/index.js?module";

const REPO = "verlab/cyd_brics5_mm1";
const BIN_PREFIX = "MM1-BLACK-denky32-";
const FLASH_ADDR = 0x10000;
const VERSION_BAUD = 9600;
const FLASH_BAUD = 115200;

let selectedPort = null;
let releases = [];
let deviceVersion = null;

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
}

async function fetchReleases() {
  const res = await fetch(`https://api.github.com/repos/${REPO}/releases`);
  if (!res.ok) throw new Error(`GitHub API ${res.status}`);
  const data = await res.json();
  releases = [];
  const sel = $("releaseSelect");
  sel.innerHTML = "";

  for (const rel of data) {
    if (rel.draft || rel.prerelease) continue;
    const asset = (rel.assets || []).find((a) =>
      a.name.startsWith(BIN_PREFIX) && a.name.endsWith(".bin")
    );
    if (!asset) continue;
    releases.push({
      tag: rel.name,
      name: rel.name,
      body: rel.body,
      url: asset.browser_download_url,
      size: asset.size,
    });
  }

  if (!releases.length) {
    sel.innerHTML = '<option value="">No releases with .bin found</option>';
    return;
  }

  for (const r of releases) {
    const opt = document.createElement("option");
    opt.value = r.tag;
    opt.textContent = `${r.tag} (${(r.size / 1024).toFixed(0)} KB)`;
    sel.appendChild(opt);
  }
  updateVersionUI();
  log(`Loaded ${releases.length} release(s).`);
}

async function readVersionFromPort(port) {
  let reader;
  let writer;
  try {
    await port.open({ baudRate: VERSION_BAUD });
    await new Promise((r) => setTimeout(r, 200));
    writer = port.writable.getWriter();
    reader = port.readable.getReader();
    await writer.write(new TextEncoder().encode("VERSION\n"));
    await writer.releaseLock();
    writer = null;

    const dec = new TextDecoder();
    let buf = "";
    const deadline = Date.now() + 2500;
    while (Date.now() < deadline) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += dec.decode(value);
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
  try {
    selectedPort = await navigator.serial.requestPort();
    log("Reading firmware version (9600 baud)...");
    log("Tip: USB must be connected; laser UART is shared on MM1.");
    deviceVersion = await readVersionFromPort(selectedPort);
    if (deviceVersion) log(`Device reports: ${deviceVersion}`);
    else log("No VERSION response — flash will still work.");
    updateVersionUI();
    $("btnFlash").disabled = !releases.length;
  } catch (e) {
    log(`Error: ${e.message || e}`);
    deviceVersion = null;
    updateVersionUI();
  }
}

async function downloadFirmware(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Download failed: ${res.status}`);
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
    log("Connect USB first.");
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

  const terminal = {
    clean: () => {},
    writeLine: (d) => log(d),
    write: (d) => log(d),
  };

  try {
    log(`Downloading ${rel.tag}...`);
    const firmware = await downloadFirmware(rel.url);
    log(`Downloaded ${firmware.byteLength} bytes.`);

    const transport = new Transport(selectedPort, true);
    const loader = new ESPLoader({
      transport,
      baudrate: FLASH_BAUD,
      terminal,
      debugLogging: false,
    });

    log("Connecting to ESP32 (115200)...");
    const chip = await loader.main();
    log(`Chip: ${chip}`);

    log("Writing flash...");
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

    log("Resetting device...");
    await loader.after("hard_reset");
    await transport.disconnect();

    log("Flash complete.");
    deviceVersion = rel.tag.replace(/^v/, "v");
    updateVersionUI();
  } catch (e) {
    log(`Flash failed: ${e.message || e}`);
  } finally {
    hideProgress();
    $("btnFlash").disabled = !releases.length;
    $("btnConnect").disabled = false;
  }
}

function init() {
  if (!("serial" in navigator)) {
    log("Web Serial not supported. Use Chrome or Edge on desktop.");
    $("btnConnect").disabled = true;
    $("btnFlash").disabled = true;
    return;
  }

  $("btnConnect").addEventListener("click", connectAndReadVersion);
  $("btnFlash").addEventListener("click", flashFirmware);
  $("releaseSelect").addEventListener("change", updateVersionUI);
  $("btnRefreshReleases").addEventListener("click", () => {
    fetchReleases().catch((e) => log(`Releases: ${e.message}`));
  });

  fetchReleases().catch((e) => log(`Releases: ${e.message}`));
}

init();
