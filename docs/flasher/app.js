/**
 * MM1-BLACK — USB installer (Web Serial + esptool-js). MIRA.
 * Release metadata from GitHub API; .bin served from ./bins/ (same origin).
 */

const REPO = "verlab/mm1-black";
const BIN_PREFIX = "MM1-BLACK-denky32-";
const FLASH_ADDR = 0x10000;
const VERSION_BAUD = 9600;
/* MM1-BLACK: ESP32 (WROOM32) + CH340 USB — Web Serial stays at one baud */
const ESP32_USB_VID = 0x1a86; /* CH340 on CYD board */
const FLASH_BAUD = 115200;
const CONNECT_TIMEOUT_MS = 22000;

const USB_PORT_FILTERS = [{ usbVendorId: ESP32_USB_VID }];

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

function flashBaud() {
  return FLASH_BAUD;
}

async function ensurePortClosed(port) {
  try {
    await port.close();
  } catch (_) {}
}

/** Set DTR/RTS one at a time — combined setSignals breaks on some Chromium builds */
async function setLineSignals(port, dtr, rts) {
  if (dtr !== undefined) {
    await port.setSignals({ dataTerminalReady: dtr });
  }
  if (rts !== undefined) {
    await port.setSignals({ requestToSend: rts });
  }
}

async function probeSerialControl(port) {
  await ensurePortClosed(port);
  try {
    await port.open({ baudRate: 115200, bufferSize: 8192 });
    await sleep(80);
    await setLineSignals(port, false, false);
    await sleep(40);
    await setLineSignals(port, false, true);
    await sleep(40);
    await setLineSignals(port, false, false);
    await ensurePortClosed(port);
    await sleep(150);
    return { ok: true };
  } catch (e) {
    await ensurePortClosed(port);
    const msg = e.message || String(e);
    return {
      ok: false,
      error:
        `USB control signals failed (${msg}). Enter download mode manually (BOOT+RST).`,
    };
  }
}

/** esptool.py classic reset — D0|R1|W100|D1|R0|W400|D0 */
async function classicBootloaderReset(port, baud = 115200) {
  await ensurePortClosed(port);
  await port.open({ baudRate: baud, bufferSize: 8192 });
  await sleep(80);
  await setLineSignals(port, false, true);
  await sleep(100);
  await setLineSignals(port, true, false);
  await sleep(400);
  await setLineSignals(port, false, undefined);
  await sleep(200);
  await ensurePortClosed(port);
  await sleep(250);
}

function withTimeout(promise, ms, label) {
  return Promise.race([
    promise,
    sleep(ms).then(() => {
      throw new Error(`${label} (timeout ${ms / 1000}s)`);
    }),
  ]);
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
  const { ESPLoader, Transport, ClassicReset } = await loadEsptool();

  const probe = await probeSerialControl(port);
  if (!probe.ok) {
    log(probe.error);
  } else {
    log("USB control signals OK.");
  }

  const attempts = [
    {
      label: "HW auto-reset",
      mode: "no_reset",
      async prep() {
        log("Toggle DTR/RTS for bootloader entry…");
        await classicBootloaderReset(port, baud);
      },
    },
    {
      label: "manual BOOT+RST",
      mode: "no_reset",
      async prep() {
        log("Hold BOOT → tap RST → release RST → release BOOT (4 s)…");
        setStatus("Download mode: BOOT + RST now…", "ok");
        await sleep(4000);
      },
    },
    {
      label: "esptool auto-reset",
      mode: "default_reset",
      async prep() {
        await sleep(300);
      },
    },
  ];

  let lastErr = null;
  for (let i = 0; i < attempts.length; i++) {
    const step = attempts[i];
    let transport;
    try {
      await step.prep();
      await ensurePortClosed(port);
      await sleep(200);

      transport = new Transport(port, false);
      const loader = new ESPLoader({
        transport,
        baudrate: baud,
        terminal,
        debugLogging: false,
        resetConstructors: {
          classicReset: (t, delay) => new ClassicReset(t, Math.max(delay, 400)),
        },
      });

      log(`Connecting ${i + 1}/${attempts.length}: ${step.label}…`);
      setStatus(`Connecting (${step.label})…`);
      const chip = await withTimeout(
        loader.main(step.mode),
        CONNECT_TIMEOUT_MS,
        `Connect timed out (${step.label})`
      );
      return { loader, transport, chip };
    } catch (e) {
      lastErr = e;
      log(`Connect failed (${step.label}): ${e.message || e}`);
      try {
        if (transport) await transport.disconnect();
      } catch (_) {}
      await ensurePortClosed(port);
      await sleep(500);
    }
  }

  throw new Error(
    (lastErr?.message || "Failed to connect with the device") +
      ". Close PlatformIO/monitor, use 115200 baud, then BOOT+RST."
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

  const baud = flashBaud();
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
  fetchReleases().catch((e) => {
    log(`Releases: ${e.message}`);
    setStatus(e.message, "err");
  });
}

init();
