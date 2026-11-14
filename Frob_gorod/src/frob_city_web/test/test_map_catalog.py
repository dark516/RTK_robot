from pathlib import Path

import pytest

from frob_city_web.map_catalog import MapCatalog, MapCatalogError


def write_pgm_map(
    root: Path,
    name: str,
    *,
    pixels: list[int],
    width: int = 2,
    height: int = 2,
    resolution: float = 0.05,
) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    image = root / f"{name}.pgm"
    image.write_text(
        f"P2\n{width} {height}\n255\n" + " ".join(str(value) for value in pixels) + "\n",
        encoding="ascii",
    )
    yaml_path = root / f"{name}.yaml"
    yaml_path.write_text(
        "\n".join((
            f"image: {image.name}",
            f"resolution: {resolution}",
            "origin: [-1.0, -2.0, 0.25]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
            "",
        )),
        encoding="utf-8",
    )
    return yaml_path


def test_catalog_lists_yaml_and_uses_opaque_ids(tmp_path):
    root = tmp_path / "maps"
    yaml_path = write_pgm_map(root, "field", pixels=[254, 0, 205, 254])

    maps = MapCatalog([root]).list_maps()

    assert [item["name"] for item in maps] == ["field"]
    assert maps[0]["path"] == str(yaml_path.resolve())
    assert maps[0]["id"] != str(yaml_path)
    assert "/" not in maps[0]["id"]


def test_catalog_rejects_unknown_or_traversal_shaped_ids(tmp_path):
    root = tmp_path / "maps"
    write_pgm_map(root, "field", pixels=[254, 0, 205, 254])
    catalog = MapCatalog([root])

    for unsafe in ("../../etc/passwd", "/etc/passwd", "missing"):
        with pytest.raises(MapCatalogError, match="unknown map"):
            catalog.preview(unsafe)


def test_preview_converts_top_left_image_to_bottom_left_occupancy(tmp_path):
    root = tmp_path / "maps"
    write_pgm_map(root, "field", pixels=[0, 254, 205, 254])
    catalog = MapCatalog([root])

    preview = catalog.preview(catalog.list_maps()[0]["id"])

    assert preview["width"] == 2
    assert preview["height"] == 2
    assert preview["resolution_m"] == 0.05
    assert preview["origin"] == {"x_m": -1.0, "y_m": -2.0, "yaw_rad": 0.25}
    assert preview["rle"] == [[-1, 1], [0, 1], [100, 1], [0, 1]]
    assert len(preview["fingerprint"]) == 64


def test_source_copy_wins_when_install_copy_has_same_name(tmp_path):
    source = tmp_path / "src" / "maps"
    installed = tmp_path / "install" / "maps"
    source_yaml = write_pgm_map(source, "field", pixels=[254], width=1, height=1)
    write_pgm_map(installed, "field", pixels=[0], width=1, height=1)

    maps = MapCatalog([source, installed]).list_maps()

    assert len(maps) == 1
    assert maps[0]["path"] == str(source_yaml.resolve())


def test_catalog_ignores_yaml_whose_image_escapes_allowlisted_root(tmp_path):
    root = tmp_path / "maps"
    root.mkdir()
    outside = tmp_path / "secret.pgm"
    outside.write_text("P2\n1 1\n255\n0\n", encoding="ascii")
    (root / "unsafe.yaml").write_text(
        "\n".join((
            "image: ../secret.pgm",
            "resolution: 0.05",
            "origin: [0, 0, 0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
        )),
        encoding="utf-8",
    )
    catalog = MapCatalog([root])
    map_id = catalog.list_maps()[0]["id"]

    with pytest.raises(MapCatalogError, match="image escaped"):
        catalog.preview(map_id)


def test_catalog_rejects_invalid_metadata(tmp_path):
    root = tmp_path / "maps"
    yaml_path = write_pgm_map(root, "field", pixels=[254, 0, 205, 254])
    yaml_path.write_text(yaml_path.read_text().replace("resolution: 0.05", "resolution: 0"))
    catalog = MapCatalog([root])

    with pytest.raises(MapCatalogError, match="resolution"):
        catalog.preview(catalog.list_maps()[0]["id"])
