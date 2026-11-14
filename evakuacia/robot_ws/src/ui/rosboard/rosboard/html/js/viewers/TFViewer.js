"use strict";

// ver 2.1
// 3D visualization for tf2_msgs/msg/TFMessage.
// Stores all transforms in memory and allows selecting frame_id and child_frame_id to display.

class TFViewer extends Space3DViewer {
  onCreate() {
    // Storage for all transforms (history)
    this.allTransforms = {}; // Map: "parent->child" -> latest transform
    this.allFrameIds = new Set(); // All frame_id values
    this.allChildFrameIds = new Set(); // All child_frame_id values
    this.visibleChildFrames = new Set(); // Which child frames to display
    
    // UI state
    this.selectedFrameId = null;
    this.axisScale = 1.0;
    this.showLinks = true;

    // Create controls container
    this.controls = $('<div></div>').css({
      "display": "flex",
      "flex-direction": "column",
      "gap": "8pt",
      "margin-bottom": "8pt",
    }).appendTo(this.card.content);

    // Frame ID selector row
    let frameRow = $('<div></div>').css({
      "display": "flex",
      "flex-wrap": "wrap",
      "gap": "8pt",
      "align-items": "center",
    }).appendTo(this.controls);

    $('<div></div>')
      .addClass("monospace")
      .css({"opacity": 0.8})
      .text("frame_id:")
      .appendTo(frameRow);

    this.frameIdSelect = $('<select></select>').css({
      "min-width": "200pt",
      "max-width": "300pt",
    }).appendTo(frameRow);

    // Child frames selector (scrollable list with checkboxes)
    let childFramesContainer = $('<div></div>').css({
      "display": "flex",
      "flex-direction": "column",
      "gap": "4pt",
      "max-height": "200pt",
      "overflow-y": "auto",
      "border": "1px solid rgba(255,255,255,0.2)",
      "padding": "8pt",
      "border-radius": "4pt",
    }).appendTo(this.controls);

    $('<div></div>')
      .addClass("monospace")
      .css({"opacity": 0.8, "margin-bottom": "4pt"})
      .text("child_frame_id (select to display):")
      .appendTo(childFramesContainer);

    this.childFramesList = $('<div></div>').appendTo(childFramesContainer);

    // Axis scale and links controls
    let settingsRow = $('<div></div>').css({
      "display": "flex",
      "flex-wrap": "wrap",
      "gap": "8pt",
      "align-items": "center",
    }).appendTo(this.controls);

    $('<div></div>')
      .addClass("monospace")
      .css({"opacity": 0.8})
      .text("axis_scale:")
      .appendTo(settingsRow);

    this.axisScaleInput = $('<input type="number" min="0.05" max="100" step="0.05"></input>')
      .css({"width": "80pt"})
      .val(this.axisScale)
      .appendTo(settingsRow);

    this.showLinksLabel = $('<label></label>')
      .css({"display": "flex", "gap": "6pt", "align-items": "center"})
      .appendTo(settingsRow);
    this.showLinksCheckbox = $('<input type="checkbox" checked></input>')
      .appendTo(this.showLinksLabel);
    $('<span></span>')
      .addClass("monospace")
      .css({"opacity": 0.8})
      .text("show links")
      .appendTo(this.showLinksLabel);

    // Event handlers
    let that = this;
    this.frameIdSelect.on("change", function() {
      that.selectedFrameId = $(this).val() || null;
      that._updateDisplay();
    });
    this.axisScaleInput.on("change", function() {
      let v = parseFloat($(this).val());
      if(!Number.isFinite(v) || v <= 0) v = 1.0;
      that.axisScale = v;
      that._updateDisplay();
    });
    this.showLinksCheckbox.on("change", function() {
      that.showLinks = !!$(this).is(":checked");
      that._updateDisplay();
    });

    super.onCreate();
  }

  // Update the list of frame_id options
  _updateFrameIdOptions() {
    let frameIds = Array.from(this.allFrameIds).sort();
    let currentValue = this.frameIdSelect.val();
    
    this.frameIdSelect.empty();
    if(frameIds.length === 0) {
      $('<option></option>').text("(no frames)").appendTo(this.frameIdSelect);
      this.selectedFrameId = null;
      return;
    }

    frameIds.forEach((fid) => {
      $('<option></option>').attr("value", fid).text(fid).appendTo(this.frameIdSelect);
    });

    // Restore selection or select first/default
    if(currentValue && frameIds.includes(currentValue)) {
      this.frameIdSelect.val(currentValue);
      this.selectedFrameId = currentValue;
    } else if(!this.selectedFrameId || !frameIds.includes(this.selectedFrameId)) {
      if(frameIds.includes("map")) {
        this.selectedFrameId = "map";
      } else {
        this.selectedFrameId = frameIds[0];
      }
      this.frameIdSelect.val(this.selectedFrameId);
    }
  }

  // Update the list of child_frame_id checkboxes
  _updateChildFramesList() {
    let childFrames = Array.from(this.allChildFrameIds).sort();
    
    this.childFramesList.empty();
    
    if(childFrames.length === 0) {
      $('<div></div>')
        .css({"opacity": 0.6, "font-style": "italic"})
        .text("(no child frames)")
        .appendTo(this.childFramesList);
      return;
    }

    // Auto-select all child frames by default (if none selected yet)
    if(this.visibleChildFrames.size === 0) {
      childFrames.forEach((cfid) => {
        this.visibleChildFrames.add(cfid);
      });
    }

    let that = this;
    childFrames.forEach((cfid) => {
      let row = $('<label></label>')
        .css({
          "display": "flex",
          "gap": "6pt",
          "align-items": "center",
          "cursor": "pointer",
        })
        .appendTo(this.childFramesList);

      let checkbox = $('<input type="checkbox"></input>')
        .prop("checked", this.visibleChildFrames.has(cfid))
        .appendTo(row);

      checkbox.on("change", function() {
        if($(this).is(":checked")) {
          that.visibleChildFrames.add(cfid);
        } else {
          that.visibleChildFrames.delete(cfid);
        }
        that._updateDisplay();
      });

      $('<span></span>')
        .addClass("monospace")
        .css({"opacity": 0.9})
        .text(cfid)
        .appendTo(row);
    });
  }

  // Store transform in memory
  _storeTransform(transform) {
    let parent = transform?.header?.frame_id;
    let child = transform?.child_frame_id;
    
    if(!parent || !child) {
      console.warn("TFViewer: Invalid transform:", transform);
      return false;
    }

    // Store in map
    let key = parent + "->" + child;
    this.allTransforms[key] = transform;

    // Update sets
    let wasNewParent = !this.allFrameIds.has(parent);
    let wasNewChild = !this.allChildFrameIds.has(child);
    this.allFrameIds.add(parent);
    this.allChildFrameIds.add(child);

    if(wasNewParent || wasNewChild) {
      console.log("TFViewer: Stored transform:", key, "New parent:", wasNewParent, "New child:", wasNewChild);
    }

    return true;
  }

  // Build transform tree for a given root frame_id
  _buildTransformTree(rootFrameId) {
    let tree = {}; // Map: child_frame_id -> {transform, position, quaternion}
    let visited = new Set();

    // Recursive function to build tree
    let buildTree = (frameId, parentPos, parentQuat) => {
      if(visited.has(frameId)) return; // Avoid cycles
      visited.add(frameId);

      // Convert parentPos to vec3 if it's an array
      let parentPosVec = Array.isArray(parentPos) 
        ? vec3.fromValues(parentPos[0], parentPos[1], parentPos[2])
        : parentPos;
      let parentPosArray = Array.isArray(parentPos) ? parentPos : [parentPos[0], parentPos[1], parentPos[2]];

      // Find all transforms where this frame is the parent
      for(let key in this.allTransforms) {
        let transform = this.allTransforms[key];
        if(transform?.header?.frame_id === frameId) {
          let childFrameId = transform?.child_frame_id;
          if(!childFrameId) continue;

          let tr = transform?.transform?.translation;
          let rot = transform?.transform?.rotation;
          if(!tr || !rot) continue;

          // Get local transform
          let localPos = vec3.fromValues(tr.x || 0, tr.y || 0, tr.z || 0);
          let localQuat = quat.fromValues(rot.x || 0, rot.y || 0, rot.z || 0, rot.w == null ? 1 : rot.w);
          quat.normalize(localQuat, localQuat);

          // Compute child position in root frame
          let childPos = vec3.create();
          let transformedLocalPos = vec3.create();
          vec3.transformQuat(transformedLocalPos, localPos, parentQuat);
          vec3.add(childPos, parentPosVec, transformedLocalPos);

          // Compute child orientation in root frame
          let childQuat = quat.create();
          quat.multiply(childQuat, parentQuat, localQuat);

          // Store in tree
          let childPosArray = [childPos[0], childPos[1], childPos[2]];
          tree[childFrameId] = {
            transform: transform,
            position: childPosArray,
            quaternion: childQuat,
            parentFrameId: frameId,
            parentPosition: parentPosArray
          };

          // Recursively process children
          buildTree(childFrameId, childPosArray, childQuat);
        }
      }
    };

    // Start building from root
    let rootPos = [0, 0, 0];
    let rootQuat = quat.fromValues(0, 0, 0, 1);
    buildTree(rootFrameId, rootPos, rootQuat);

    return tree;
  }

  // Render a frame's axes
  _renderFrameAxes(vertices, colors, position, quaternion, scale) {
    let ex = vec3.fromValues(1, 0, 0);
    let ey = vec3.fromValues(0, 1, 0);
    let ez = vec3.fromValues(0, 0, 1);

    let vx = vec3.create();
    let vy = vec3.create();
    let vz = vec3.create();

    vec3.transformQuat(vx, ex, quaternion);
    vec3.transformQuat(vy, ey, quaternion);
    vec3.transformQuat(vz, ez, quaternion);
    vec3.scale(vx, vx, scale);
    vec3.scale(vy, vy, scale);
    vec3.scale(vz, vz, scale);

    // X axis (red)
    this._pushLine(vertices, colors, position,
      [position[0] + vx[0], position[1] + vx[1], position[2] + vx[2]],
      [1.0, 0.2, 0.2, 1.0]);
    
    // Y axis (green)
    this._pushLine(vertices, colors, position,
      [position[0] + vy[0], position[1] + vy[1], position[2] + vy[2]],
      [0.2, 1.0, 0.2, 1.0]);
    
    // Z axis (cyan)
    this._pushLine(vertices, colors, position,
      [position[0] + vz[0], position[1] + vz[1], position[2] + vz[2]],
      [0.2, 0.6, 1.0, 1.0]);
  }

  // Push a line to vertices/colors arrays
  _pushLine(vertices, colors, p0, p1, rgba) {
    vertices.push(p0[0], p0[1], p0[2]);
    vertices.push(p1[0], p1[1], p1[2]);
    colors.push(rgba[0], rgba[1], rgba[2], rgba[3]);
    colors.push(rgba[0], rgba[1], rgba[2], rgba[3]);
  }

  // Update the 3D display
  _updateDisplay() {
    if(!this.selectedFrameId) {
      console.log("TFViewer: No frame selected");
      this.draw([]);
      return;
    }

    console.log("TFViewer: Updating display for frame_id:", this.selectedFrameId, 
                "Visible child frames:", Array.from(this.visibleChildFrames));

    let vertices = [];
    let colors = [];

    let scale = this.axisScale;
    if(!Number.isFinite(scale) || scale <= 0) scale = 1.0;

    // Render root frame axes at origin
    let rootPos = [0, 0, 0];
    let rootQuat = quat.fromValues(0, 0, 0, 1);
    this._renderFrameAxes(vertices, colors, rootPos, rootQuat, scale);

    // Build transform tree for selected frame_id
    let tree = this._buildTransformTree(this.selectedFrameId);
    console.log("TFViewer: Built tree with", Object.keys(tree).length, "child frames:", Object.keys(tree));

    // Render visible child frames
    let renderedCount = 0;
    for(let childFrameId in tree) {
      if(!this.visibleChildFrames.has(childFrameId)) {
        console.log("TFViewer: Skipping", childFrameId, "(not visible)");
        continue;
      }

      let frameData = tree[childFrameId];
      let position = frameData.position;
      let quaternion = frameData.quaternion;

      console.log("TFViewer: Rendering", childFrameId, "at", position);

      // Render axes for this child frame
      this._renderFrameAxes(vertices, colors, position, quaternion, scale);

      // Draw link from parent to child if enabled
      if(this.showLinks) {
        let parentPos = frameData.parentPosition;
        this._pushLine(vertices, colors, parentPos, position, [0.7, 0.7, 0.7, 0.6]);
      }
      
      renderedCount++;
    }

    console.log("TFViewer: Rendered", renderedCount, "child frames. Total vertices:", vertices.length / 3);

    this.draw([
      {type: "lines", data: new Float32Array(vertices), colors: new Float32Array(colors)},
    ]);
  }

  // Process incoming TF message
  onData(msg) {
    this._lastMsg = msg;
    this.card.title.text(msg._topic_name);

    let transforms = msg.transforms || [];
    if(!Array.isArray(transforms) || transforms.length === 0) {
      this.warn("TFMessage has no transforms[]");
      this.draw([]);
      return;
    }

    // Store all transforms
    let hasNewFrames = false;
    let previousChildFramesCount = this.allChildFrameIds.size;
    
    for(let i = 0; i < transforms.length; i++) {
      if(this._storeTransform(transforms[i])) {
        hasNewFrames = true;
      }
    }

    // Update UI if new frames appeared
    if(hasNewFrames) {
      this._updateFrameIdOptions();
      
      // Auto-select newly added child frames
      if(this.allChildFrameIds.size > previousChildFramesCount) {
        this.allChildFrameIds.forEach((cfid) => {
          if(!this.visibleChildFrames.has(cfid)) {
            this.visibleChildFrames.add(cfid);
          }
        });
      }
      
      this._updateChildFramesList();
    }

    // Update display
    this._updateDisplay();
  }
}

TFViewer.friendlyName = "TF (3D)";
TFViewer.supportedTypes = [
  "tf2_msgs/msg/TFMessage",
];
TFViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(TFViewer);
