/**
 * MM1-BLACK installer (Web Serial + esptool-js).
 * Firmware list from GitHub Releases — requires a public repo (like Tasmota).
 */

const REPO = "verlab/cyd_brics5_mm1";
const BIN_PREFIX = "MM1-BLACK-denky32-";
const FLASH_ADDR = 0x10000;
const VERSION_BAUD = 9600;

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

function updateUI() {
  const rel = releases.find((r) => r.tag === $("releaseSelect").value);
  const cmp = $("versionCompare");
  const dv = normalizeDeviceVer(deviceVersion);

  $("deviceVersion").textContent = deviceVersion || "—";

  if (rel && dv) {
    const rv = parseTagVer(rel.tag);
    if (rv) {
      const c = compareVer(rv, dv);
      if (c > 0)
        cmp.innerHTML =
          '<span class="compare-newer">Newer release available.</span>';
      else if (c < 0)
        cmp.innerHTML =
          '<span class="compare-older">Device is newer than selected build.</span>';
      else cmp.textContent = "Device matches selected release.";
      return;
    }
  }
  cmp.textContent = rel ? "" : "";

  const canInstall =
    releases.length > 0 &&
    $("releaseSelect").value &&
    $("ackFlash").checked &&
    "serial" in navigator;
  $("btnInstall").disabled = !canInstall;
}

async function fetchReleases() {
  const sel = $("releaseSelect");
  sel.innerHTML = '<option value="">Loading…</option>';
  sel.disabled = true;
  setStatus("Loading releases from GitHub…");
  releases = [];

  const res = await fetch(`https://api.github.com/repos/${REPO}/releases`, {
    headers: { Accept: "application/vnd.github+json" },
  });

  if (res.status === 404) {
    sel.innerHTML = '<option value="">Repository not accessible</option>';
    setStatus(
      "Cannot load releases — the repo must be public (Settings → Change visibility).",
      "err"
    );
    log(
      "GitHub API returned 404. Private repos block the browser from listing/downloading release assets."
    );
    log(`Make ${REPO} public, or use pio run -t upload locally.`);
    updateUI();
    return;
  }

  if (!res.ok) {
    throw new Error(`GitHub API HTTP ${res.status}`);
  }

  const data = await res.json();
  for (const rel of data) {
    if (rel.draft || rel.prerelease) continue;
    const tag = rel.tag_name || rel.name;
    const asset = (rel.assets || []).find(
      (a) => a.name.startsWith(BIN_PREFIX) && a.name.endsWith(".bin")
    );
    if (!asset) continue;
    releases.push({
      tag,
      name: rel.name || tag,
      url: asset.browser_download_url,
      size: asset.size,
    });
  }

  sel.innerHTML = "";
  if (!releases.length) {
    sel.innerHTML =
      '<option value="">No MM1-BLACK-denky32-*.bin on Releases yet</option>';
    setStatus("No firmware assets found. Tag a release (v*).", "err");
    updateUI();
    return;
  }

  for (const r of releases) {
    const opt = document.createElement("option");
    opt.value = r.tag;
    opt.textContent = `${r.tag} (${(r.size / 1024).toFixed(0)} KB)`;
    sel.appendChild(opt);
  }
  sel.disabled = false;
  setStatus(`${releases.length} release(s) from GitHub.`, "ok");
  log(`Loaded ${releases.length} release(s).`);
  updateUI();
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

async function ensurePort() {
  if (selectedPort) return selectedPort;
  setStatus("Choose the USB serial port…");
  selectedPort = await navigator.serial.requestPort();
  return selectedPort;
}

async function readInstalledVersion() {
  if (!("serial" in navigator)) return;

  try {
    const port = await ensurePort();
    setStatus("Reading installed version…");
    deviceVersion = await readVersionFromPort(port);
    selectedPort = port;
    if (deviceVersion) {
      log(`Installed: ${deviceVersion}`);
      setStatus(`Installed firmware: ${deviceVersion}`, "ok");
    } else {
      log("No VERSION response (wrong port or device busy).");
      setStatus("Version not read — you can still install.", "ok");
    }
    updateUI();
  } catch (e) {
    if (e.name !== "NotFoundError") {
      log(`Read version: ${e.message || e}`);
      setStatus(e.message || String(e), "err");
    }
    updateUI();
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
  log(`Downloading…`);
  const res = await fetch(url);
  if (!res.ok) {
    if (res.status === 404)
      throw new Error(
        "Download 404 — repo must be public for browser access to release files"
      );
    throw new Error(`Download HTTP ${res.status}`);
  }
  return new Uint8Array(await res.arrayBuffer());
}

async function installFirmware() {
  const tag = $("releaseSelect").value;
  const rel = releases.find((r) => r.tag === tag);
  if (!rel) {
    setStatus("Select a firmware release.", "err");
    return;
  }
  if (!$("ackFlash").checked) {
    setStatus("Confirm the checkbox first.", "err");
    return;
  }

  const baud = parseInt($("flashBaud").value, 10) || 115200;
  $("btnInstall").disabled = true;
  $("btnReadVersion").disabled = true;
  setProgress(0);
  setStatus("Installing… do not unplug USB.");

  const terminal = {
    clean: () => {},
    writeLine: (d) => log(d),
    write: (d) => log(d),
  };

  try {
    const port = await ensurePort();
    log(`Port selected. Flashing ${rel.tag} @ ${baud} baud…`);

    const firmware = await downloadFirmware(rel.url);
    log(`Downloaded ${(firmware.byteLength / 1024).toFixed(0)} KB.`);

    const { ESPLoader, Transport } = await loadEsptool();
    const transport = new Transport(port, true);
    const loader = new ESPLoader({
      transport,
      baudrate: baud,
      terminal,
      debugLogging: false,
    });

    log("Connecting to ESP32…");
    const chip = await loader.main();
    log(`Chip: ${chip}`);

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

    log("Resetting…");
    await loader.after("hard_reset");
    await transport.disconnect();

    deviceVersion = rel.tag.replace(/^v/, "");
    log("Install complete.");
    setStatus(`Installed ${rel.tag} successfully.`, "ok");
    updateUI();
  } catch (e) {
    if (e.name === "NotFoundError") setStatus("No port selected.", "err");
    else {
      log(`Install failed: ${e.message || e}`);
      setStatus(`Install failed: ${e.message || e}`, "err");
    }
  } finally {
    hideProgress();
    $("btnReadVersion").disabled = false;
    updateUI();
  }
}

function init() {
  if (!("serial" in navigator)) {
    $("noSerial").classList.remove("hidden");
    $("btnInstall").disabled = true;
    $("btnReadVersion").disabled = true;
    return;
  }

  $("btnInstall").addEventListener("click", installFirmware);
  $("btnReadVersion").addEventListener("click", readInstalledVersion);
  $("releaseSelect").addEventListener("change", updateUI);
  $("ackFlash").addEventListener("change", updateUI);
  $("flashBaud").addEventListener("change", updateUI);

  fetchReleases().catch((e) => {
    log(`Releases: ${e.message}`);
    setStatus(e.message, "err");
  });
}

init();
