(function (root, factory) {
  "use strict";
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.FrobCalibration = api;
  if (typeof document !== "undefined") {
    document.addEventListener("DOMContentLoaded", () => api.start());
  }
})(typeof window !== "undefined" ? window : null, function () {
  "use strict";

  const FIELD = Object.freeze({gridSize: 5, sizeM: 4, cellM: 0.8});
  const PLATFORM_CELLS = new Set(["B2", "D2", "B4", "D4"]);

  function clone(value) { return JSON.parse(JSON.stringify(value)); }

  function fieldToMap(point, transform) {
    const c = Math.cos(Number(transform.yaw_rad)), s = Math.sin(Number(transform.yaw_rad));
    return [Number(transform.x_m) + c * Number(point[0]) - s * Number(point[1]),
            Number(transform.y_m) + s * Number(point[0]) + c * Number(point[1])];
  }

  function createFixedPreset(name, mapSource) {
    const cells = [];
    for (let row = 0; row < FIELD.gridSize; row += 1) {
      for (let column = 0; column < FIELD.gridSize; column += 1) {
        const id = `${String.fromCharCode(65 + column)}${row + 1}`;
        const nominal = [(column + 0.5) * FIELD.cellM, (row + 0.5) * FIELD.cellM];
        cells.push({id, column, row, kind: PLATFORM_CELLS.has(id) ? "platform" : "road",
          navigable: !PLATFORM_CELLS.has(id), navigation_point: {nominal, active: [...nominal], provenance: "preset"}});
      }
    }
    return {schema_version: 1, id: null, revision: 0, name, lifecycle: "draft",
      map_source: clone(mapSource), field: {preset: "frob-city-5x5-v1", grid_size: 5,
        size_m: [4, 4], cell_size_m: 0.8, field_to_map: {x_m: 0, y_m: 0, yaw_rad: 0}},
      geometry: {cells, roads: [], platforms: [], observation_points: [], sign_sites: []}, calibrations: {}};
  }

  function setYawAroundFieldCenter(document, yaw) {
    const transform = document.field.field_to_map;
    const centre = fieldToMap([FIELD.sizeM / 2, FIELD.sizeM / 2], transform);
    const half = FIELD.sizeM / 2, c = Math.cos(Number(yaw)), s = Math.sin(Number(yaw));
    transform.x_m = centre[0] - c * half + s * half;
    transform.y_m = centre[1] - s * half - c * half;
    transform.yaw_rad = Number(yaw);
    return document;
  }

  function mapSelectionMode(calibrationDocument, mapId) {
    if (!calibrationDocument || calibrationDocument.map_source?.id === mapId) return "load";
    return "new-draft";
  }

  function decodeRle(runs, expectedLength) {
    const values = [];
    for (const run of runs || []) {
      for (let index = 0; index < Number(run[1]); index += 1) values.push(Number(run[0]));
      if (values.length > expectedLength) throw new Error("RLE exceeds map dimensions");
    }
    if (values.length !== expectedLength) throw new Error("RLE does not match map dimensions");
    return values;
  }

  function findPoint(document, reference) {
    const [kind, id, rawIndex] = String(reference).split(":");
    const geometry = document.geometry;
    if (kind === "cell") return geometry.cells.find(item => item.id === id)?.navigation_point || null;
    if (kind === "sign") return geometry.sign_sites.find(item => item.id === id)?.position || null;
    if (kind === "observation") return geometry.observation_points.find(item => item.id === id)?.position || null;
    if (kind === "road") return geometry.roads.find(item => item.id === id)?.points?.[Number(rawIndex)] || null;
    return null;
  }

  function applyLocalPointEdit(document, reference, mapPoint) {
    const target = findPoint(document, reference);
    if (!target) throw new Error(`Unknown editable point: ${reference}`);
    target.active = [Number(mapPoint[0]), Number(mapPoint[1])];
    target.provenance = "drag";
    return document;
  }

  function roadReference(document, reference) {
    const [kind, id, rawIndex] = String(reference).split(":");
    const road = kind === "road" ? document.geometry.roads.find(item => item.id === id) : null;
    const index = Number(rawIndex);
    if (!road || !Number.isInteger(index) || index < 0 || index >= road.points.length) {
      throw new Error(`Unknown road point: ${reference}`);
    }
    return {road, id, index};
  }

  function insertRoadPoint(document, reference) {
    const {road, id, index} = roadReference(document, reference);
    if (index >= road.points.length - 1) throw new Error("Cannot insert after the route endpoint");
    const transform = document.field.field_to_map, left = road.points[index], right = road.points[index + 1];
    const active = point => point.provenance === "preset" ? fieldToMap(point.nominal, transform) : point.active.map(Number);
    const leftActive = active(left), rightActive = active(right);
    road.points.splice(index + 1, 0, {
      nominal: [(Number(left.nominal[0]) + Number(right.nominal[0])) / 2,
        (Number(left.nominal[1]) + Number(right.nominal[1])) / 2],
      active: [(leftActive[0] + rightActive[0]) / 2, (leftActive[1] + rightActive[1]) / 2],
      provenance: "drag",
    });
    return `road:${id}:${index + 1}`;
  }

  function removeRoadPoint(document, reference) {
    const {road, index} = roadReference(document, reference);
    if (index === 0 || index === road.points.length - 1) throw new Error("Cannot remove a route endpoint");
    road.points.splice(index, 1);
    return document;
  }

  function validateDraftClientSide(document) {
    const errors = [];
    if (document?.field?.grid_size !== FIELD.gridSize || document?.field?.size_m?.[0] !== FIELD.sizeM ||
        document?.field?.size_m?.[1] !== FIELD.sizeM || document?.geometry?.cells?.length !== 25) {
      errors.push({code: "FIXED_GRID", path: "field", message: "Поле должно оставаться 5×5 и 4×4 м"});
    }
    const numbers = [document?.field?.field_to_map?.x_m, document?.field?.field_to_map?.y_m,
      document?.field?.field_to_map?.yaw_rad];
    if (numbers.some(value => !Number.isFinite(Number(value)))) {
      errors.push({code: "NON_FINITE", path: "field.field_to_map", message: "Координаты должны быть конечными"});
    }
    return errors;
  }

  function start() {
    const $ = id => document.getElementById(id);
    const canvas = $("calibration-canvas");
    const context = canvas.getContext("2d");
    const state = {
      maps: null, map: null, mapPixels: null, document: null, drafts: null,
      tool: "global", tab: "field", selection: null, validation: null,
      layers: {grid: true, navigation: true, roads: true, platforms: true,
        observations: true, signs: true, residuals: true},
      view: {x: 2, y: 2, ppm: 130}, drag: null, measure: null,
      undo: [], redo: [], dirty: false, saving: false, saveTimer: null,
      renderQueued: false,
    };

    const LABELS = {
      footprint: "Контур робота", robot_radius_m: "Радиус робота", stopping_distance_m: "Тормозной путь",
      command_latency_s: "Задержка команд", perception_latency_s: "Задержка восприятия",
      normal_speed_mps: "Рабочая скорость", cautious_speed_mps: "Осторожная скорость",
      maximum_speed_mps: "Максимальная скорость", watchdog_timeout_s: "Таймаут watchdog",
      clearance_margin_m: "Запас до препятствий", camera_to_base: "Положение камеры",
      image_width_px: "Ширина кадра", image_height_px: "Высота кадра", intrinsics: "Матрица камеры",
      distortion: "Коэффициенты дисторсии", sign_dimensions: "Размеры знаков",
      confidence_threshold: "Порог уверенности", range_uncertainty_m: "Ошибка дальности",
      bearing_uncertainty_rad: "Ошибка направления", map_frame: "Фрейм карты", odom_frame: "Фрейм одометрии",
      base_frame: "Базовый фрейм", camera_frame: "Фрейм камеры", maximum_pose_covariance: "Допустимая ковариация",
      maximum_jump_m: "Максимальный скачок", map_resolution_m: "Разрешение карты", stale_timeout_s: "Таймаут данных",
      start_cell: "Стартовая клетка", start_heading_rad: "Начальное направление",
      completion_stop_count: "Количество остановок", stop_dwell_s: "Время остановки",
      parking_prior: "Предположение о парковке", route_tolerance_m: "Допуск маршрута",
      observation_tolerance_rad: "Допуск наблюдения", map_topic: "Топик карты", odom_topic: "Топик одометрии",
      detections_topic: "Топик детекций", nav2_action: "Действие Nav2", motion_permit_topic: "Разрешение движения",
      cmd_vel_raw_topic: "Сырые команды", cmd_vel_topic: "Финальные команды",
      sole_cmd_vel_authority: "Единственный источник команд", critical_topic_timeout_s: "Таймаут критичных топиков",
    };
    const TAB_NAMES = {field: "Поле", robot_safety: "Робот и безопасность", camera_signs: "Камера и знаки",
      localization: "Локализация", mission: "Миссия", ros_nav2: "ROS / Nav2", readiness: "Готовность"};

    async function api(path, options = {}) {
      const request = {...options, headers: {...(options.headers || {})}};
      if (request.body && typeof request.body !== "string") {
        request.body = JSON.stringify(request.body);
        request.headers["Content-Type"] = "application/json";
      }
      let response;
      try { response = await fetch(path, request); }
      catch (error) { setConnected(false); throw error; }
      setConnected(true);
      const payload = await response.json();
      if (!response.ok || payload.ok === false) {
        const error = new Error(payload.errors?.map(item => item.message).join("; ") || `HTTP ${response.status}`);
        error.issues = payload.errors || [];
        error.status = response.status;
        throw error;
      }
      return payload.data;
    }

    function setConnected(connected) { $("connection-banner").classList.toggle("visible", !connected); }
    function toast(message, kind = "") {
      const item = document.createElement("div");
      item.className = `toast ${kind}`; item.textContent = message; $("toast-stack").append(item);
      setTimeout(() => item.remove(), 4500);
    }
    function displayError(error) { toast(error.message || String(error), "error"); }

    function option(value, text, kind) {
      const item = document.createElement("option"); item.value = value; item.textContent = text;
      if (kind) item.dataset.kind = kind; return item;
    }

    async function bootstrap() {
      try {
        const [maps, drafts] = await Promise.all([api("/api/calibration/maps"), api("/api/calibration/drafts")]);
        state.maps = maps; state.drafts = drafts; populateMaps(); populateDrafts();
        if (drafts.drafts.length) await loadDraft(drafts.drafts[0].id);
        else {
          const initial = maps.live.available ? "live" : maps.files[0]?.id;
          if (initial) { $("map-picker").value = initial; await loadMap(initial, true); }
          renderInspector(); renderTab(); updateActions();
        }
      } catch (error) { displayError(error); }
    }

    function populateMaps() {
      const picker = $("map-picker"); picker.replaceChildren();
      const live = state.maps.live;
      picker.append(option("live", live.available ? `LIVE /map · ${Number(live.age_s).toFixed(1)} s` :
        (live.stale ? `STALE /map · ${Number(live.age_s).toFixed(1)} s` : "LIVE /map · нет данных"), "live"));
      picker.options[0].disabled = !live.available;
      for (const item of state.maps.files) picker.append(option(item.id, `FILE · ${item.name}`, "file"));
      if (!live.available && !state.maps.files.length) picker.append(option("", "Нет доступных карт"));
    }

    function populateDrafts(selectedId = state.document?.id) {
      const picker = $("draft-picker"); picker.replaceChildren(option("", "Новый черновик"));
      for (const item of state.drafts?.drafts || []) {
        const suffix = state.drafts.active?.id === item.id ? " · ACTIVE" : "";
        picker.append(option(item.id, `${item.name}${suffix}`));
      }
      picker.value = selectedId || "";
    }

    async function loadMap(mapId, fit) {
      if (!mapId) return;
      const map = await api(`/api/calibration/map-preview?id=${encodeURIComponent(mapId)}`);
      state.map = map; state.mapPixels = buildMapCanvas(map); $("canvas-empty").classList.add("hidden");
      const isLive = map.source.kind === "live";
      $("source-badge").className = `source-badge ${isLive ? "live" : "file"}`;
      $("source-badge").textContent = isLive ? "LIVE /map" : "MAP FILE";
      $("source-detail").textContent = isLive ? `Возраст: ${Number(state.maps.live.age_s).toFixed(2)} s` : map.source.path;
      if (fit) fitMap(); else scheduleDraw();
    }

    async function selectMap(mapId) {
      if (!mapId) return;
      const previousSourceId = state.document?.map_source?.id;
      if (mapSelectionMode(state.document, mapId) === "new-draft") {
        if (state.dirty) {
          await saveDraft(true);
          if (state.dirty) {
            $("map-picker").value = previousSourceId;
            return toast("Не удалось сохранить текущий черновик", "error");
          }
        }
        await loadMap(mapId, true);
        clearTimeout(state.saveTimer);
        state.document = null; state.validation = null; state.undo = []; state.redo = [];
        populateDrafts(); setDraftState("empty", "НЕ СОЗДАН"); renderAll();
        toast("Карта выбрана. Нажмите «Новый», чтобы создать отдельный черновик.");
        return;
      }
      await loadMap(mapId, true);
    }

    function buildMapCanvas(map) {
      const pixels = decodeRle(map.rle, Number(map.width) * Number(map.height));
      const result = document.createElement("canvas"); result.width = map.width; result.height = map.height;
      const imageContext = result.getContext("2d"), image = imageContext.createImageData(map.width, map.height);
      for (let y = 0; y < map.height; y += 1) for (let x = 0; x < map.width; x += 1) {
        const value = pixels[y * map.width + x], screenY = map.height - 1 - y;
        const offset = (screenY * map.width + x) * 4;
        const shade = value < 0 ? 73 : value >= 65 ? 10 : 220;
        image.data[offset] = shade; image.data[offset + 1] = value < 0 ? 78 : shade;
        image.data[offset + 2] = value < 0 ? 76 : shade; image.data[offset + 3] = 255;
      }
      imageContext.putImageData(image, 0, 0); return result;
    }

    async function loadDraft(draftId) {
      try {
        state.document = await api(`/api/calibration/drafts/${encodeURIComponent(draftId)}`);
        state.undo = []; state.redo = []; state.dirty = false; state.validation = state.document.validation;
        $("draft-name").value = state.document.name; populateDrafts(draftId);
        const sourceId = state.document.map_source.id; $("map-picker").value = sourceId;
        await loadMap(sourceId, true); setDraftState("saved"); renderAll();
      } catch (error) { displayError(error); }
    }

    async function createDraft() {
      const selected = $("map-picker").selectedOptions[0];
      if (!selected?.value) return toast("Сначала выберите карту", "warning");
      const mapSource = {kind: selected.dataset.kind, id: selected.value};
      try {
        state.document = await api("/api/calibration/drafts", {method: "POST",
          body: {name: $("draft-name").value, map_source: mapSource}});
        if (state.map?.source?.id !== state.document.map_source.id) {
          await loadMap(state.document.map_source.id, true);
        }
        state.validation = null; state.undo = []; state.redo = []; state.dirty = false;
        state.drafts = await api("/api/calibration/drafts"); populateDrafts(state.document.id);
        setDraftState("saved"); updateActions(); renderAll(); toast("Черновик создан");
      } catch (error) { displayError(error); }
    }

    async function saveDraft(quiet = false) {
      if (!state.document?.id || state.saving || !state.dirty) return;
      state.saving = true; clearTimeout(state.saveTimer);
      state.document.name = $("draft-name").value.trim() || "Калибровка поля";
      try {
        state.document = await api(`/api/calibration/drafts/${encodeURIComponent(state.document.id)}`, {
          method: "PUT", body: {revision: state.document.revision, document: state.document},
        });
        state.dirty = false; setDraftState("saved");
        state.drafts = await api("/api/calibration/drafts"); populateDrafts(state.document.id);
        if (!quiet) toast("Черновик сохранён");
      } catch (error) {
        if (error.status === 409) toast("Черновик изменён в другом окне. Перезагрузите его.", "error");
        else displayError(error);
      } finally { state.saving = false; updateActions(); }
    }

    async function validateDraft() {
      if (!state.document) return;
      if (state.dirty) await saveDraft(true);
      try {
        state.validation = await api("/api/calibration/validate", {method: "POST",
          body: {id: state.document.id, revision: state.document.revision}});
        renderValidation(); renderTab(); updateActions();
        toast(state.validation.activation_ready ? "Калибровка готова к активации" :
          `Проверка завершена: ${state.validation.errors.length} ошибок, ${state.validation.blockers.length} блокеров`,
          state.validation.valid ? "warning" : "error");
      } catch (error) { displayError(error); }
    }

    async function activateDraft() {
      if (!state.document || !state.validation?.activation_ready) return;
      if (!window.confirm("Активировать эту ревизию? Предыдущая активная калибровка будет сохранена в backup.")) return;
      try {
        state.document = await api("/api/calibration/activate", {method: "POST",
          body: {id: state.document.id, revision: state.document.revision}});
        state.drafts = await api("/api/calibration/drafts"); populateDrafts(state.document.id);
        setDraftState("saved", "ACTIVE"); toast("Калибровка активирована");
      } catch (error) { displayError(error); }
    }

    function pushUndo() {
      if (!state.document) return;
      state.undo.push(clone(state.document)); if (state.undo.length > 50) state.undo.shift();
      state.redo = []; updateUndoButtons();
    }
    function undo() {
      if (!state.undo.length || !state.document) return;
      state.redo.push(clone(state.document)); state.document = state.undo.pop(); markDirty(false); renderAll();
    }
    function redo() {
      if (!state.redo.length || !state.document) return;
      state.undo.push(clone(state.document)); state.document = state.redo.pop(); markDirty(false); renderAll();
    }
    function updateUndoButtons() { $("undo-edit").disabled = !state.undo.length; $("redo-edit").disabled = !state.redo.length; }
    function markDirty(autosave = true) {
      if (!state.document) return;
      state.dirty = true; state.validation = null; setDraftState("dirty"); updateActions(); scheduleDraw(); renderTab();
      if (autosave) { clearTimeout(state.saveTimer); state.saveTimer = setTimeout(() => saveDraft(true), 1400); }
    }
    function setDraftState(kind, text) {
      const badge = $("draft-state"); badge.className = `draft-state ${kind}`;
      badge.textContent = text || (kind === "dirty" ? "НЕ СОХРАНЁН" : "СОХРАНЁН");
    }
    function updateActions() {
      const ready = Boolean(state.document?.id);
      $("save-draft").disabled = !ready || !state.dirty || state.saving;
      $("validate-draft").disabled = !ready || state.saving;
      $("activate-draft").disabled = !ready || state.dirty || !state.validation?.activation_ready;
    }

    function activePoint(record, transform) {
      return record.provenance === "preset" ? fieldToMap(record.nominal, transform) : record.active.map(Number);
    }
    function overlay() {
      if (!state.document) return null;
      const transform = state.document.field.field_to_map, geometry = state.document.geometry;
      const cells = geometry.cells.map(cell => {
        const nominal = fieldToMap(cell.navigation_point.nominal, transform);
        const active = activePoint(cell.navigation_point, transform);
        return {...cell, nominal_map: nominal, active_map: active,
          residual_m: Math.hypot(active[0] - nominal[0], active[1] - nominal[1])};
      });
      const roads = geometry.roads.map(road => ({...road, handles: road.points.map((point, index) => {
        const nominal = fieldToMap(point.nominal, transform), active = activePoint(point, transform);
        return {reference: `road:${road.id}:${index}`, nominal, active,
          residual_m: Math.hypot(active[0] - nominal[0], active[1] - nominal[1])};
      })}));
      const platforms = geometry.platforms.map(platform => {
        const polygon = platform.polygon;
        const nominal = polygon.nominal.map(point => fieldToMap(point, transform));
        return {...platform, nominal_map: nominal,
          active_map: polygon.provenance === "preset" ? nominal : polygon.active.map(point => point.map(Number))};
      });
      const mapItems = (items, kind) => items.map(item => {
        const nominal = fieldToMap(item.position.nominal, transform), active = activePoint(item.position, transform);
        return {...item, reference: `${kind}:${item.id}`, nominal_map: nominal, active_map: active,
          residual_m: Math.hypot(active[0] - nominal[0], active[1] - nominal[1])};
      });
      return {cells, roads, platforms, observations: mapItems(geometry.observation_points, "observation"),
        signs: mapItems(geometry.sign_sites, "sign")};
    }

    function worldToScreen(point) {
      const rect = canvas.getBoundingClientRect();
      return [rect.width / 2 + (point[0] - state.view.x) * state.view.ppm,
              rect.height / 2 - (point[1] - state.view.y) * state.view.ppm];
    }
    function screenToWorld(clientX, clientY) {
      const rect = canvas.getBoundingClientRect();
      return [state.view.x + (clientX - rect.left - rect.width / 2) / state.view.ppm,
              state.view.y - (clientY - rect.top - rect.height / 2) / state.view.ppm];
    }
    function mapCorners(map) {
      const origin = map.origin, width = map.width * map.resolution_m, height = map.height * map.resolution_m;
      return [[0, 0], [width, 0], [width, height], [0, height]].map(point => fieldToMap(point, origin));
    }
    function fitMap() {
      if (!state.map) return;
      const points = mapCorners(state.map), xs = points.map(point => point[0]), ys = points.map(point => point[1]);
      const rect = canvas.getBoundingClientRect(); state.view.x = (Math.min(...xs) + Math.max(...xs)) / 2;
      state.view.y = (Math.min(...ys) + Math.max(...ys)) / 2;
      state.view.ppm = Math.max(20, Math.min((rect.width - 70) / (Math.max(...xs) - Math.min(...xs)),
        (rect.height - 70) / (Math.max(...ys) - Math.min(...ys)))); scheduleDraw();
    }

    function scheduleDraw() {
      if (state.renderQueued) return; state.renderQueued = true;
      requestAnimationFrame(() => { state.renderQueued = false; draw(); });
    }
    function line(points, color, width = 1, dash = []) {
      if (!points.length) return; context.beginPath();
      points.forEach((point, index) => { const screen = worldToScreen(point); index ? context.lineTo(...screen) : context.moveTo(...screen); });
      context.strokeStyle = color; context.lineWidth = width; context.setLineDash(dash); context.stroke(); context.setLineDash([]);
    }
    function polygon(points, fill, stroke, width = 1) {
      if (!points.length) return; context.beginPath(); points.forEach((point, index) => {
        const screen = worldToScreen(point); index ? context.lineTo(...screen) : context.moveTo(...screen);
      }); context.closePath(); context.fillStyle = fill; context.fill(); context.strokeStyle = stroke;
      context.lineWidth = width; context.stroke();
    }
    function pointGlyph(point, color, radius, shape = "circle") {
      const [x, y] = worldToScreen(point); context.save(); context.translate(x, y);
      context.fillStyle = color; context.strokeStyle = "#071013"; context.lineWidth = 2;
      context.beginPath();
      if (shape === "diamond") { context.rotate(Math.PI / 4); context.rect(-radius, -radius, radius * 2, radius * 2); }
      else context.arc(0, 0, radius, 0, Math.PI * 2);
      context.fill(); context.stroke(); context.restore();
    }

    function draw() {
      const rect = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
      if (canvas.width !== Math.round(rect.width * dpr) || canvas.height !== Math.round(rect.height * dpr)) {
        canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
      }
      context.setTransform(dpr, 0, 0, dpr, 0, 0); context.clearRect(0, 0, rect.width, rect.height);
      context.fillStyle = "#060a0c"; context.fillRect(0, 0, rect.width, rect.height);
      if (state.map && state.mapPixels) {
        const origin = worldToScreen([state.map.origin.x_m, state.map.origin.y_m]); context.save();
        context.translate(...origin); context.rotate(-Number(state.map.origin.yaw_rad || 0));
        context.scale(state.view.ppm * state.map.resolution_m, state.view.ppm * state.map.resolution_m);
        context.globalAlpha = .78; context.imageSmoothingEnabled = false;
        context.drawImage(state.mapPixels, 0, -state.map.height); context.restore();
      }
      const data = overlay(); if (!data) return;
      const transform = state.document.field.field_to_map;
      if (state.layers.roads) for (const road of data.roads) line(road.handles.map(item => item.active), "rgba(86,216,207,.62)", 3);
      if (state.layers.platforms) for (const platform of data.platforms) polygon(platform.active_map, "rgba(255,148,88,.2)", "#ff9458", 2);
      if (state.layers.grid) {
        for (let index = 0; index <= FIELD.gridSize; index += 1) {
          const offset = index * FIELD.cellM;
          line([fieldToMap([offset, 0], transform), fieldToMap([offset, FIELD.sizeM], transform)], "rgba(86,216,207,.74)", index % 5 === 0 ? 2 : 1, [6, 5]);
          line([fieldToMap([0, offset], transform), fieldToMap([FIELD.sizeM, offset], transform)], "rgba(86,216,207,.74)", index % 5 === 0 ? 2 : 1, [6, 5]);
        }
        context.font = "700 10px 'DejaVu Sans Mono', monospace"; context.textAlign = "center"; context.textBaseline = "middle";
        for (const cell of data.cells) { const centre = fieldToMap(cell.navigation_point.nominal, transform), screen = worldToScreen(centre);
          context.fillStyle = cell.navigable ? "rgba(237,242,235,.62)" : "#ff9458"; context.fillText(cell.id, screen[0], screen[1] - 13); }
      }
      if (state.layers.residuals) {
        for (const cell of data.cells) if (cell.residual_m > .001) line([cell.nominal_map, cell.active_map], "#ff6862", 1.5, [4, 3]);
        for (const item of [...data.observations, ...data.signs]) if (item.residual_m > .001) line([item.nominal_map, item.active_map], "#ff6862", 1.5, [4, 3]);
      }
      if (state.layers.navigation) for (const cell of data.cells) if (cell.navigable) pointGlyph(cell.active_map, "#f0b540", 4);
      if (state.layers.observations) for (const item of data.observations) pointGlyph(item.active_map, "#75d79b", 4);
      if (state.layers.signs) for (const item of data.signs) pointGlyph(item.active_map, "#ff6862", 4, "diamond");
      if (state.tool === "global") {
        const handle = fieldToMap([FIELD.sizeM, FIELD.sizeM + .28], transform); line([fieldToMap([FIELD.sizeM, FIELD.sizeM], transform), handle], "#f0b540", 1);
        pointGlyph(handle, "#f0b540", 7);
      }
      if (state.selection) {
        const selected = selectionPoint(data, state.selection.reference);
        if (selected) { const [x, y] = worldToScreen(selected); context.strokeStyle = "#ffffff"; context.lineWidth = 2;
          context.strokeRect(x - 10, y - 10, 20, 20); }
      }
      if (state.measure?.start && state.measure?.current) line([state.measure.start, state.measure.current], "#ffffff", 1, [3, 3]);
      const residuals = data.cells.map(item => item.residual_m).concat(data.signs.map(item => item.residual_m));
      $("residual-readout").textContent = `${(residuals.reduce((a, b) => a + b, 0) / Math.max(1, residuals.length)).toFixed(3)} m`;
      $("map-scale").textContent = `${(100 / state.view.ppm).toFixed(2)} m / 100 px`;
    }

    function selectionPoint(data, reference) {
      const [kind, id, rawIndex] = reference.split(":");
      if (kind === "cell") return data.cells.find(item => item.id === id)?.active_map;
      if (kind === "sign") return data.signs.find(item => item.id === id)?.active_map;
      if (kind === "observation") return data.observations.find(item => item.id === id)?.active_map;
      if (kind === "road") return data.roads.find(item => item.id === id)?.handles?.[Number(rawIndex)]?.active;
      if (kind === "platform") return data.platforms.find(item => item.id === id)?.active_map?.[Number(rawIndex)];
      return null;
    }

    function hitTest(clientX, clientY) {
      const data = overlay(); if (!data) return null;
      const candidates = [];
      if (state.layers.navigation) for (const item of data.cells) if (item.navigable) candidates.push({reference: `cell:${item.id}`, point: item.active_map, label: item.id});
      if (state.layers.signs) for (const item of data.signs) candidates.push({reference: item.reference, point: item.active_map, label: item.id});
      if (state.layers.observations) for (const item of data.observations) candidates.push({reference: item.reference, point: item.active_map, label: item.id});
      if (state.layers.roads) for (const road of data.roads) road.handles.forEach(item => candidates.push({reference: item.reference, point: item.active, label: road.id}));
      if (state.layers.platforms) for (const platform of data.platforms) platform.active_map.forEach((point, index) => candidates.push({reference: `platform:${platform.id}:${index}`, point, label: platform.id}));
      let best = null;
      for (const candidate of candidates) { const screen = worldToScreen(candidate.point), distance = Math.hypot(screen[0] - clientX, screen[1] - clientY);
        if (distance <= 14 && (!best || distance < best.distance)) best = {...candidate, distance}; }
      return best;
    }

    function editPlatformVertex(reference, mapPoint) {
      const [, id, rawIndex] = reference.split(":"), platform = state.document.geometry.platforms.find(item => item.id === id);
      if (!platform) return; if (platform.polygon.provenance === "preset") {
        platform.polygon.active = platform.polygon.nominal.map(point => fieldToMap(point, state.document.field.field_to_map));
      }
      platform.polygon.active[Number(rawIndex)] = mapPoint.map(Number); platform.polygon.provenance = "drag";
    }
    function editReference(reference, mapPoint) {
      if (reference.startsWith("platform:")) editPlatformVertex(reference, mapPoint);
      else applyLocalPointEdit(state.document, reference, mapPoint);
    }

    function pointerDown(event) {
      if (!state.map) return; canvas.setPointerCapture(event.pointerId);
      const world = screenToWorld(event.clientX, event.clientY);
      if (state.tool === "pan") state.drag = {type: "pan", client: [event.clientX, event.clientY], view: {...state.view}};
      else if (state.tool === "measure") { state.measure = {start: world, current: world}; state.drag = {type: "measure"}; }
      else if (!state.document) return toast("Создайте черновик", "warning");
      else if (state.tool === "local") {
        const hit = hitTest(event.offsetX, event.offsetY); if (!hit) return;
        pushUndo(); state.selection = hit; state.drag = {type: "local", reference: hit.reference}; renderInspector();
      } else if (state.tool === "global") {
        pushUndo(); const transform = clone(state.document.field.field_to_map);
        const rotationHandle = worldToScreen(fieldToMap([FIELD.sizeM, FIELD.sizeM + .28], transform));
        const rect = canvas.getBoundingClientRect(), sx = event.clientX - rect.left, sy = event.clientY - rect.top;
        if (Math.hypot(sx - rotationHandle[0], sy - rotationHandle[1]) < 18) {
          const centre = fieldToMap([FIELD.sizeM / 2, FIELD.sizeM / 2], transform);
          state.drag = {type: "rotate", centre, startAngle: Math.atan2(world[1] - centre[1], world[0] - centre[0]), transform};
        } else state.drag = {type: "global", start: world, transform};
      }
      scheduleDraw();
    }

    function pointerMove(event) {
      const world = screenToWorld(event.clientX, event.clientY);
      $("cursor-coordinates").textContent = `X ${world[0].toFixed(3)} / Y ${world[1].toFixed(3)}`;
      if (!state.drag) return;
      if (state.drag.type === "pan") {
        state.view.x = state.drag.view.x - (event.clientX - state.drag.client[0]) / state.view.ppm;
        state.view.y = state.drag.view.y + (event.clientY - state.drag.client[1]) / state.view.ppm;
      } else if (state.drag.type === "measure") state.measure.current = world;
      else if (state.drag.type === "local") editReference(state.drag.reference, world);
      else if (state.drag.type === "global") {
        state.document.field.field_to_map.x_m = state.drag.transform.x_m + world[0] - state.drag.start[0];
        state.document.field.field_to_map.y_m = state.drag.transform.y_m + world[1] - state.drag.start[1];
      } else if (state.drag.type === "rotate") {
        const angle = Math.atan2(world[1] - state.drag.centre[1], world[0] - state.drag.centre[0]);
        setYawAroundFieldCenter(state.document, state.drag.transform.yaw_rad + angle - state.drag.startAngle);
      }
      scheduleDraw(); renderInspector();
    }

    function pointerUp() {
      if (!state.drag) return; const edited = !["pan", "measure"].includes(state.drag.type);
      state.drag = null; if (edited) markDirty(); scheduleDraw();
    }

    function renderInspector() {
      const target = $("inspector-content");
      if (!state.document) { target.innerHTML = '<div class="empty-inspector">Создайте черновик, чтобы редактировать геометрию.</div>'; return; }
      const transform = state.document.field.field_to_map;
      let selection = "";
      if (state.selection) {
        const point = selectionPoint(overlay(), state.selection.reference);
        let routeActions = "";
        if (state.selection.reference.startsWith("road:")) {
          const {road, index} = roadReference(state.document, state.selection.reference);
          routeActions = `<div class="input-grid route-actions">
            <button class="button ghost" type="button" id="insert-route-point" ${index >= road.points.length - 1 ? "disabled" : ""}>+ ТОЧКА ПОСЛЕ</button>
            <button class="button ghost" type="button" id="remove-route-point" ${index === 0 || index === road.points.length - 1 ? "disabled" : ""}>− УДАЛИТЬ ТОЧКУ</button></div>`;
        }
        selection = `<article class="selection-card"><h3>${escapeHtml(state.selection.label || state.selection.reference)}</h3>
          <p>${escapeHtml(state.selection.reference)} · map frame</p><div class="input-grid">
          <label>X<input type="number" step="0.001" data-selected-axis="0" value="${point[0].toFixed(4)}"></label>
          <label>Y<input type="number" step="0.001" data-selected-axis="1" value="${point[1].toFixed(4)}"></label></div>
          ${routeActions}<button class="button ghost" type="button" id="reset-selection">СБРОСИТЬ ЛОКАЛЬНУЮ ПОПРАВКУ</button></article>`;
      }
      target.innerHTML = `<form class="field-form"><div class="inspector-note">Сплошная операция — только перенос и поворот. Локальные ручки не искривляют систему координат.</div>
        <div class="input-grid"><label>Tₓ<div class="input-with-unit"><input type="number" step="0.001" data-transform="x_m" value="${Number(transform.x_m).toFixed(4)}"><span>m</span></div></label>
        <label>Tᵧ<div class="input-with-unit"><input type="number" step="0.001" data-transform="y_m" value="${Number(transform.y_m).toFixed(4)}"><span>m</span></div></label></div>
        <label>Поворот<div class="input-with-unit"><input type="number" step="0.1" data-transform="yaw_deg" value="${(Number(transform.yaw_rad) * 180 / Math.PI).toFixed(2)}"><span>deg</span></div></label>
        ${selection}</form>`;
      target.querySelectorAll("[data-transform]").forEach(input => input.addEventListener("change", event => {
        const value = Number(event.target.value); if (!Number.isFinite(value)) return;
        pushUndo(); const key = event.target.dataset.transform;
        if (key === "yaw_deg") setYawAroundFieldCenter(state.document, value * Math.PI / 180);
        else state.document.field.field_to_map[key] = value; markDirty(); renderAll();
      }));
      target.querySelectorAll("[data-selected-axis]").forEach(input => input.addEventListener("change", event => {
        const value = Number(event.target.value); if (!Number.isFinite(value)) return;
        const point = selectionPoint(overlay(), state.selection.reference).slice(); point[Number(event.target.dataset.selectedAxis)] = value;
        pushUndo(); editReference(state.selection.reference, point); markDirty(); renderAll();
      }));
      target.querySelector("#insert-route-point")?.addEventListener("click", () => {
        try { pushUndo(); const reference = insertRoadPoint(state.document, state.selection.reference);
          state.selection = {reference, label: state.selection.label}; markDirty(); renderAll(); }
        catch (error) { displayError(error); }
      });
      target.querySelector("#remove-route-point")?.addEventListener("click", () => {
        try { const [, id, rawIndex] = state.selection.reference.split(":"); pushUndo();
          removeRoadPoint(state.document, state.selection.reference);
          state.selection = {reference: `road:${id}:${Math.max(0, Number(rawIndex) - 1)}`, label: state.selection.label};
          markDirty(); renderAll(); }
        catch (error) { displayError(error); }
      });
      target.querySelector("#reset-selection")?.addEventListener("click", resetSelection);
    }

    function resetSelection() {
      if (!state.selection) return; pushUndo(); const reference = state.selection.reference;
      if (reference.startsWith("platform:")) {
        const [, id] = reference.split(":"), platform = state.document.geometry.platforms.find(item => item.id === id);
        platform.polygon.active = clone(platform.polygon.nominal); platform.polygon.provenance = "preset";
      } else { const point = findPoint(state.document, reference); point.active = clone(point.nominal); point.provenance = "preset"; }
      markDirty(); renderAll();
    }

    function escapeHtml(value) { const node = document.createElement("span"); node.textContent = String(value); return node.innerHTML; }
    function settingValue(value) {
      if (value === null || value === undefined) return "";
      return typeof value === "object" ? JSON.stringify(value) : String(value);
    }
    function parseSetting(raw, previous) {
      if (!raw.trim()) return null;
      if (typeof previous === "number") { const value = Number(raw); if (!Number.isFinite(value)) throw new Error("Ожидалось число"); return value; }
      if (typeof previous === "boolean") return raw === "true";
      if (Array.isArray(previous) || (previous && typeof previous === "object") || /^[\[{]/.test(raw.trim())) return JSON.parse(raw);
      return raw;
    }

    function renderTab() {
      $("tab-title").textContent = TAB_NAMES[state.tab];
      const target = $("tab-content");
      if (!state.document) { target.innerHTML = '<div class="empty-inspector">Нет открытого черновика.</div>'; return; }
      if (state.tab === "field") {
        const data = overlay(), corrected = data.cells.filter(item => item.residual_m > .001).length +
          data.signs.filter(item => item.residual_m > .001).length;
        target.innerHTML = `<div class="stat-grid"><div class="stat"><span>ПОЛЕ</span><b>5 × 5</b></div>
          <div class="stat"><span>ДОРОЖНЫЕ КЛЕТКИ</span><b>21</b></div><div class="stat"><span>ПЛАТФОРМЫ</span><b>${data.platforms.length}</b></div>
          <div class="stat"><span>МЕСТА ЗНАКОВ</span><b>${data.signs.length}</b></div><div class="stat"><span>ДОРОЖНЫЕ СВЯЗИ</span><b>${data.roads.length}</b></div>
          <div class="stat"><span>ТОЧКИ ОБЗОРА</span><b>${data.observations.length}</b></div><div class="stat"><span>ЛОКАЛЬНЫЕ ПОПРАВКИ</span><b>${corrected}</b></div>
          <div class="stat"><span>ПРЕСЕТ</span><b>v1</b></div></div>`;
      } else if (state.tab === "readiness") renderReadiness(target);
      else renderSettings(target, state.tab);
    }

    function renderSettings(target, groupName) {
      const group = state.document.calibrations[groupName] || {};
      target.innerHTML = `<div class="setting-list">${Object.entries(group).map(([key, setting]) => `<article class="setting-row">
        <label>${escapeHtml(LABELS[key] || key)}<div class="input-with-unit"><input data-setting-value="${key}" value="${escapeHtml(settingValue(setting.value))}"><span>${escapeHtml(setting.units || "—")}</span></div></label>
        <div class="setting-meta"><input data-setting-evidence="${key}" value="${escapeHtml(setting.evidence || "")}" placeholder="Источник / заметка" aria-label="Источник значения">
        <select data-setting-status="${key}" aria-label="Статус значения">${["missing","draft","validated","active"].map(status => `<option value="${status}" ${setting.status === status ? "selected" : ""}>${status}</option>`).join("")}</select></div></article>`).join("")}</div>`;
      target.querySelectorAll("[data-setting-value]").forEach(input => input.addEventListener("change", event => {
        const key = event.target.dataset.settingValue, setting = group[key];
        try { pushUndo(); setting.value = parseSetting(event.target.value, setting.value); setting.status = setting.value === null ? "missing" : "draft";
          setting.provenance = "operator"; setting.measured_at = new Date().toISOString(); markDirty(); renderTab(); }
        catch (error) { displayError(error); renderTab(); }
      }));
      target.querySelectorAll("[data-setting-evidence]").forEach(input => input.addEventListener("change", event => {
        pushUndo(); group[event.target.dataset.settingEvidence].evidence = event.target.value; markDirty();
      }));
      target.querySelectorAll("[data-setting-status]").forEach(select => select.addEventListener("change", event => {
        pushUndo(); group[event.target.dataset.settingStatus].status = event.target.value; markDirty();
      }));
    }

    function renderReadiness(target) {
      const counts = {missing: 0, draft: 0, validated: 0, active: 0};
      for (const group of Object.values(state.document.calibrations)) for (const setting of Object.values(group)) {
        counts[setting.status] = (counts[setting.status] || 0) + 1;
      }
      target.innerHTML = `<div class="stat-grid">${Object.entries(counts).map(([key, value]) =>
        `<div class="stat"><span>${key.toUpperCase()}</span><b>${value}</b></div>`).join("")}
        <div class="stat"><span>ОШИБКИ ГЕОМЕТРИИ</span><b>${state.validation?.errors?.length ?? "—"}</b></div>
        <div class="stat"><span>БЛОКЕРЫ АКТИВАЦИИ</span><b>${state.validation?.blockers?.length ?? "—"}</b></div></div>`;
    }

    function renderValidation() {
      const target = $("validation-list"), summary = $("validation-summary");
      if (!state.validation) { summary.className = "validation-summary"; summary.textContent = "НЕ ЗАПУЩЕНА";
        target.innerHTML = "<p>Сохраните и проверьте черновик. Геометрические ошибки и физические блокеры будут показаны отдельно.</p>"; return; }
      const groups = [["error", state.validation.errors], ["warning", state.validation.warnings], ["blocker", state.validation.blockers]];
      const issues = groups.flatMap(([kind, items]) => (items || []).map(item => ({kind, ...item})));
      summary.className = `validation-summary ${state.validation.activation_ready ? "pass" : "fail"}`;
      summary.textContent = state.validation.activation_ready ? "READY" : `${state.validation.errors.length} ERR / ${state.validation.blockers.length} BLOCK`;
      target.innerHTML = issues.length ? issues.map(item => `<div class="issue ${item.kind}" data-object-id="${escapeHtml(item.object_id || "")}">
        <b>${item.kind.toUpperCase()}</b><span>${escapeHtml(item.message)}<code>${escapeHtml(item.path)}</code></span></div>`).join("") : "<p>Ошибок и предупреждений нет.</p>";
      target.querySelectorAll("[data-object-id]").forEach(item => item.addEventListener("click", () => selectObject(item.dataset.objectId)));
    }

    function selectObject(objectId) {
      if (!objectId || !state.document) return;
      if (state.document.geometry.cells.some(item => item.id === objectId)) state.selection = {reference: `cell:${objectId}`, label: objectId};
      else if (state.document.geometry.sign_sites.some(item => item.id === objectId)) state.selection = {reference: `sign:${objectId}`, label: objectId};
      else if (state.document.geometry.observation_points.some(item => item.id === objectId)) state.selection = {reference: `observation:${objectId}`, label: objectId};
      else return;
      state.tool = "local"; syncTools(); renderInspector(); scheduleDraw();
    }
    function renderAll() { renderInspector(); renderTab(); renderValidation(); updateUndoButtons(); updateActions(); scheduleDraw(); }

    function setTool(tool) { state.tool = tool; state.measure = null; syncTools(); scheduleDraw(); }
    function syncTools() {
      document.querySelectorAll("[data-tool]").forEach(button => button.classList.toggle("active", button.dataset.tool === state.tool));
      $("tool-readout").textContent = {pan: "MAP PAN", global: "GLOBAL FIT", local: "LOCAL EDIT", measure: "MEASURE"}[state.tool];
    }

    $("map-picker").addEventListener("change", event => selectMap(event.target.value).catch(displayError));
    $("draft-picker").addEventListener("change", event => {
      if (event.target.value) loadDraft(event.target.value); else { state.document = null; state.validation = null; renderAll(); }
    });
    $("new-draft").addEventListener("click", createDraft); $("save-draft").addEventListener("click", () => saveDraft(false));
    $("validate-draft").addEventListener("click", validateDraft); $("activate-draft").addEventListener("click", activateDraft);
    $("draft-name").addEventListener("change", () => { if (state.document) { pushUndo(); state.document.name = $("draft-name").value; markDirty(); } });
    document.querySelectorAll("[data-tool]").forEach(button => button.addEventListener("click", () => setTool(button.dataset.tool)));
    document.querySelectorAll("[data-yaw]").forEach(button => button.addEventListener("click", () => {
      if (!state.document) return toast("Создайте черновик", "warning"); pushUndo();
      setYawAroundFieldCenter(state.document, Number(button.dataset.yaw) * Math.PI / 180); markDirty(); renderAll();
    }));
    document.querySelectorAll("[data-tab]").forEach(button => button.addEventListener("click", () => {
      state.tab = button.dataset.tab; document.querySelectorAll("[data-tab]").forEach(item => item.classList.toggle("active", item === button)); renderTab();
    }));
    document.querySelectorAll("[data-layer]").forEach(input => input.addEventListener("change", () => { state.layers[input.dataset.layer] = input.checked; scheduleDraw(); }));
    $("fit-map").addEventListener("click", fitMap); $("undo-edit").addEventListener("click", undo); $("redo-edit").addEventListener("click", redo);
    canvas.addEventListener("pointerdown", pointerDown); canvas.addEventListener("pointermove", pointerMove);
    canvas.addEventListener("pointerup", pointerUp); canvas.addEventListener("pointercancel", pointerUp);
    canvas.addEventListener("wheel", event => { event.preventDefault(); const before = screenToWorld(event.clientX, event.clientY);
      state.view.ppm = Math.max(20, Math.min(1200, state.view.ppm * (event.deltaY < 0 ? 1.12 : .89)));
      const after = screenToWorld(event.clientX, event.clientY); state.view.x += before[0] - after[0]; state.view.y += before[1] - after[1]; scheduleDraw(); }, {passive: false});
    window.addEventListener("keydown", event => {
      if (["INPUT", "SELECT", "TEXTAREA"].includes(event.target.tagName)) return;
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") { event.preventDefault(); event.shiftKey ? redo() : undo(); return; }
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "y") { event.preventDefault(); redo(); return; }
      const tool = {h: "pan", g: "global", l: "local", m: "measure"}[event.key.toLowerCase()]; if (tool) setTool(tool);
      if (event.key.toLowerCase() === "f") fitMap(); if (event.key === "Escape") { state.drag = null; state.measure = null; scheduleDraw(); }
    });
    new ResizeObserver(scheduleDraw).observe(canvas); syncTools(); renderAll(); bootstrap();
  }

  return {FIELD, createFixedPreset, fieldToMap, setYawAroundFieldCenter, mapSelectionMode, decodeRle,
    applyLocalPointEdit, insertRoadPoint, removeRoadPoint, validateDraftClientSide, start};
});
