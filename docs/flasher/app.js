/**
 * MM1-BLACK da MIRA — USB installer (Web Serial + esptool-js).
 * Release metadata from GitHub API; .bin served from ./bins/ (same origin).
 */

const REPO = "verlab/mm1-black";
const BIN_PREFIX = "MM1-BLACK-denky32-";
const FLASH_ADDR = 0x10000;
const VERSION_BAUD = 9600;
/* CH340 (CYD USB): Web Serial cannot reconfigure baud mid-session — stay ≤115200 */
const CH340_VID = 0x1a86;
const WEB_MAX_BAUD = 115200;
const CONNECT_ROUNDS = 3;

const USB_PORT_FILTERS = [
  { usbVendorId: 0x1a86 }, /* CH340 */
  { usbVendorId: 0x10c4 }, /* CP210x */
  { usbVendorId: 0x0403 }, /* FTDI */
];

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

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

function isCh340(port) {
  const info = port.getInfo();
  return info.usbVendorId === CH340_VID;
}

function effectiveFlashBaud(port, selectedBaud) {
  if (selectedBaud <= WEB_MAX_BAUD) return selectedBaud;
  if (isCh340(port)) {
    log(
      `CH340: Web Serial keeps one baud for the whole session — using ${WEB_MAX_BAUD} instead of ${selectedBaud}.`
    );
    return WEB_MAX_BAUD;
  }
  log(`Using ${WEB_MAX_BAUD} (browser flash limit).`);
  return WEB_MAX_BAUD;
}

async function ensurePortClosed(port) {
  try {
    await port.close();
  } catch (_) {}
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

function localBinUrl(fileName) {
  return new URL(`bins/${fileName}`, window.location.href).href;
}

async function fetchReleases() {
  const sel = $("releaseSelect");
  sel.innerHTML = '<option value="">Loading…</option>';
  sel.disabled = true;
  setStatus("Loading releases…");
  releases = [];

  const res = await fetch(`https://api.github.com/repos/${REPO}/releases`, {
    headers: { Accept: "application/vnd.github+json" },
  });

  if (res.status === 404) {
    sel.innerHTML = '<option value="">Repository not accessible</option>';
    setStatus("Cannot load releases — check that the repo is public.", "err");
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
      fileName: asset.name,
      url: localBinUrl(asset.name),
      size: asset.size,
    });
  }

  sel.innerHTML = "";
  if (!releases.length) {
    sel.innerHTML =
      '<option value="">No MM1-BLACK-denky32-*.bin on Releases yet</option>';
    setStatus("No firmware on Releases. Tag v* and re-run Pages deploy.", "err");
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
  setStatus(`${releases.length} release(s) ready.`, "ok");
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

async function requestPort() {
  setStatus("Choose the USB serial port…");
  try {
    selectedPort = await navigator.serial.requestPort({
      filters: USB_PORT_FILTERS,
    });
  } catch (e) {
    if (e.name === "NotFoundError") throw e;
    selectedPort = await navigator.serial.requestPort();
  }
  return selectedPort;
}

async function readInstalledVersion() {
  if (!("serial" in navigator)) return;

  try {
    const port = await requestPort();
    setStatus("Reading installed version…");
    deviceVersion = await readVersionFromPort(port);
    selectedPort = null;
    if (deviceVersion) {
      log(`Installed: ${deviceVersion}`);
      setStatus(`Installed firmware: ${deviceVersion}`, "ok");
    } else {
      log("No VERSION response (wrong port or device busy).");
      setStatus("Version not read — you can still install.", "ok");
    }
    updateUI();
  } catch (e) {
    selectedPort = null;
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
    /* 0.6.x expects Uint8Array for writeFlash; 0.5.x compress path breaks on binary buffers. */
    esptoolModule = await import(
      "https://cdn.jsdelivr.net/npm/esptool-js@0.6.0/+esm"
    );
  }
  return esptoolModule;
}

async function downloadFirmware(rel) {
  log(`Downloading ${rel.fileName}…`);
  let res = await fetch(rel.url);
  if (!res.ok) {
    throw new Error(
      `Firmware file not on this site (HTTP ${res.status}). ` +
        "Re-deploy Pages after a new Release, or wait a few minutes."
    );
  }
  return new Uint8Array(await res.arrayBuffer());
}

async function connectLoader(port, baud, terminal) {
  const { ESPLoader, Transport } = await loadEsptool();
  const modes = ["default_reset", "no_reset"];
  let lastErr = null;

  for (let round = 0; round < CONNECT_ROUNDS; round++) {
    for (const mode of modes) {
      let transport;
      try {
        await ensurePortClosed(port);
        await sleep(250);
        transport = new Transport(port, false);
        const loader = new ESPLoader({
          transport,
          baudrate: baud,
          terminal,
          debugLogging: false,
        });

        if (mode === "no_reset") {
          log(
            "Manual boot: hold BOOT → tap RST → release RST → release BOOT, then connecting…"
          );
          setStatus("Hold BOOT, tap RST — entering download mode…", "ok");
          await sleep(2200);
        } else if (round > 0) {
          log(`Retry ${round + 1}/${CONNECT_ROUNDS} (auto-reset)…`);
          await sleep(600);
        }

        log(`Connecting (${mode})…`);
        const chip = await loader.main(mode);
        return { loader, transport, chip };
      } catch (e) {
        lastErr = e;
        log(`Connect failed (${mode}): ${e.message || e}`);
        try {
          if (transport) await transport.disconnect();
        } catch (_) {}
        await ensurePortClosed(port);
        await sleep(400);
      }
    }
  }

  throw (
    lastErr ||
    new Error(
      "Failed to connect — close other serial tools, try 115200 baud, use BOOT+RST."
    )
  );
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

  const baudSel = parseInt($("flashBaud").value, 10) || WEB_MAX_BAUD;
  $("btnInstall").disabled = true;
  $("btnReadVersion").disabled = true;
  setProgress(0);
  setStatus("Downloading firmware…");

  const terminal = {
    clean: () => {},
    writeLine: (d) => log(d),
    write: (d) => log(d),
  };

  try {
    const firmware = await downloadFirmware(rel);
    log(`Downloaded ${(firmware.byteLength / 1024).toFixed(0)} KB.`);

    setStatus("Select USB port and flash…");
    selectedPort = null;
    const port = await requestPort();
    const baud = effectiveFlashBaud(port, baudSel);
    const info = port.getInfo();
    if (info.usbVendorId) {
      log(
        `USB 0x${info.usbVendorId.toString(16)}:0x${(info.usbProductId || 0).toString(16)}`
      );
    }
    log(`Flashing ${rel.tag} @ ${baud} baud…`);

    setStatus("Connecting to ESP32 bootloader…");
    const { loader, transport, chip } = await connectLoader(
      port,
      baud,
      terminal
    );
    log(`Chip: ${chip}`);

    setStatus("Writing flash… do not unplug USB.");
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
    /* writeFlash leaves stub with flashDeflFinish(false) — must reboot to run new firmware */
    if (loader.IS_STUB) await loader.flashDeflFinish(true);
    await new Promise((r) => setTimeout(r, 400));
    try {
      await loader.after("hard_reset");
    } catch (e) {
      log(`hard_reset: ${e.message || e}`);
      try {
        await loader.softReset(false);
      } catch (_) {}
    }
    await new Promise((r) => setTimeout(r, 600));
    try {
      await transport.disconnect();
    } catch (_) {}
    selectedPort = null;

    deviceVersion = rel.tag.replace(/^v/, "");
    log("Install complete.");
    setStatus(`Installed ${rel.tag} successfully.`, "ok");
    updateUI();
  } catch (e) {
    selectedPort = null;
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
