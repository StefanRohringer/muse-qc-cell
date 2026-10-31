"use strict";

const STEP_MS = 2000;
let parts = [];
let index = 0;
let timer = null;

const labelEl = document.getElementById("run-label");
const cardEl = document.getElementById("card");
const counterEl = document.getElementById("counter");
const playBtn = document.getElementById("btn-play");
const pauseBtn = document.getElementById("btn-pause");
const prevBtn = document.getElementById("btn-prev");
const nextBtn = document.getElementById("btn-next");

function countsShown() {
  let good = 0;
  let bad = 0;
  for (let i = 0; i <= index && i < parts.length; i++) {
    if (parts[i].verdict === "good") {
      good++;
    } else if (parts[i].verdict === "bad") {
      bad++;
    }
  }
  return { good, bad };
}

function render() {
  if (parts.length === 0) {
    return;
  }
  const part = parts[index];
  const badge = part.verdict === "good"
    ? '<span class="badge good">GOOD</span>'
    : '<span class="badge bad">BAD</span>';
  let boxLine = "";
  if (part.box) {
    boxLine = "<p>Box: " + escapeHtml(String(part.box)) + "</p>";
  }
  let retryLine = "";
  if (part.retries !== undefined && part.retries !== null) {
    retryLine = "<p>Retries: " + escapeHtml(String(part.retries)) + "</p>";
  }
  cardEl.innerHTML =
    "<img src=\"" + encodeURI("data/" + part.image) + "\" alt=\"Close-up of part " +
    (index + 1) + "\">" +
    "<p>Part " + (index + 1) + " of " + parts.length +
    " — " + escapeHtml(formatTimeUtc(part.time)) + "</p>" +
    "<p>Levers: " + escapeHtml(String(part.levers)) + " " + badge + "</p>" +
    boxLine + retryLine;
  const counts = countsShown();
  counterEl.textContent =
    "good " + counts.good + ", bad " + counts.bad +
    " (part " + (index + 1) + " of " + parts.length + ")";
}

function formatTimeUtc(iso) {
  const date = new Date(String(iso));
  if (isNaN(date.getTime())) {
    return String(iso);
  }
  const pad = (n) => String(n).padStart(2, "0");
  return pad(date.getUTCHours()) + ":" + pad(date.getUTCMinutes()) + ":" +
    pad(date.getUTCSeconds()) + " UTC";
}

function escapeHtml(text) {
  return text
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function stop() {
  if (timer !== null) {
    clearInterval(timer);
    timer = null;
  }
}

function play() {
  stop();
  timer = setInterval(() => {
    if (index < parts.length - 1) {
      index++;
      render();
    } else {
      stop();
    }
  }, STEP_MS);
}

playBtn.addEventListener("click", play);
pauseBtn.addEventListener("click", stop);
prevBtn.addEventListener("click", () => {
  stop();
  if (index > 0) {
    index--;
    render();
  }
});
nextBtn.addEventListener("click", () => {
  stop();
  if (index < parts.length - 1) {
    index++;
    render();
  }
});

fetch("data/run.json")
  .then((response) => {
    if (!response.ok) {
      throw new Error("missing");
    }
    return response.json();
  })
  .then((data) => {
    if (!data.parts || data.parts.length === 0) {
      labelEl.textContent = "no run data";
      return;
    }
    parts = data.parts;
    index = 0;
    labelEl.textContent = "Replay of a real run on " + data.date;
    render();
  })
  .catch(() => {
    labelEl.textContent = "no run data";
  });
