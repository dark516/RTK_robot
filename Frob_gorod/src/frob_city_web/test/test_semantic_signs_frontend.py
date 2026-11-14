import json
from pathlib import Path
import shutil
import subprocess

import pytest


APP_JS = (
    Path(__file__).resolve().parents[1]
    / "frob_city_web"
    / "static"
    / "app.js"
)


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_known_target_is_rendered_once_at_exact_physical_face():
    script = """
const { buildSemanticSigns } = require(%s);
const field = {sign_sites: [{id: "SIGN_B2_NW", position: [0.86, 1.54]}]};
const state = {
  targets: [{
    target_id: "BUS_STOP:SIGN_B2_NW:FACE_W",
    kind: {value: 6, label: "BUS_STOP"},
    site_id: "SIGN_B2_NW",
    face_id: "FACE_W",
    state: {label: "CONFIRMED"},
    sign_pose: {x_m: 1.17, y_m: -0.44}
  }],
  known_signs: [{
    site_id: "SIGN_B2_NW",
    face_id: "FACE_W",
    approach_id: "",
    sign_class: {value: 6, label: "BUS_STOP"}
  }]
};
const records = buildSemanticSigns(field, state, point => point);
if (records.length !== 1) throw new Error(`expected one sign, got ${records.length}`);
if (records[0].source !== "known_sign") throw new Error("known placement must win");
if (records[0].site !== "SIGN_B2_NW" || records[0].face !== "FACE_W") {
  throw new Error("physical identity was lost");
}
""" % json.dumps(str(APP_JS))
    completed = subprocess.run(
        ["node", "-e", script],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
