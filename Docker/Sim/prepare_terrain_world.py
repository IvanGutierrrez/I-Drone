#!/usr/bin/env python3
"""Generate a Gazebo world with a heightmap from a GeoTIFF terrain raster."""

import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


def run_cmd(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"Command failed ({' '.join(cmd)}):\n{result.stdout}\n{result.stderr}"
        )
    return result.stdout


def command_exists(name):
    result = subprocess.run(["which", name], capture_output=True, text=True)
    return result.returncode == 0


def is_pow2_plus_1(value):
    if value < 3:
        return False
    n = value - 1
    return (n & (n - 1)) == 0


def nearest_pow2_plus_1(max_size):
    n = 1
    while (n + 1) < max_size:
        n <<= 1
    return n + 1


_ALBEDO_NATIVE_SENTINEL = object()


def parse_albedo_dim(value):
    """Return (width, height) or _ALBEDO_NATIVE_SENTINEL for native/max/full/original."""
    if not value:
        return _ALBEDO_NATIVE_SENTINEL

    normalized = value.strip().lower()
    if normalized in {"native", "max", "full", "original"}:
        return _ALBEDO_NATIVE_SENTINEL

    if "x" in normalized:
        width_str, height_str = normalized.split("x", 1)
        return max(1, int(width_str)), max(1, int(height_str))

    dim = max(1, int(normalized))
    return dim, dim


_MAX_ALBEDO_DIM = 16384  # GPU-safe texture size limit


def compute_native_albedo_dim(albedo_info_text, extent_x_m, extent_y_m):
    """Compute albedo pixel dimensions from the source pixel size, capped at _MAX_ALBEDO_DIM."""
    pixel_match = re.search(
        r"Pixel Size\s*=\s*\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)",
        albedo_info_text,
    )
    if not pixel_match:
        return None
    px = abs(float(pixel_match.group(1)))
    py = abs(float(pixel_match.group(2)))
    w = int(extent_x_m / px)
    h = int(extent_y_m / py)
    if max(w, h) > _MAX_ALBEDO_DIM:
        scale = _MAX_ALBEDO_DIM / max(w, h)
        w = max(1, int(w * scale))
        h = max(1, int(h * scale))
    return w, h


def parse_gdalinfo(info_text):
    size_match = re.search(r"Size is\s+(\d+),\s*(\d+)", info_text)
    if not size_match:
        raise ValueError("Could not parse raster size from gdalinfo output")

    pixel_match = re.search(
        r"Pixel Size\s*=\s*\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)",
        info_text,
    )
    if not pixel_match:
        raise ValueError("Could not parse pixel size from gdalinfo output")

    minmax_match = re.search(
        r"Computed Min/Max=\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)",
        info_text,
    )
    if not minmax_match:
        raise ValueError(
            "Could not parse terrain min/max from gdalinfo output. "
            "Ensure gdalinfo supports -mm for this file."
        )

    raster_x = int(size_match.group(1))
    raster_y = int(size_match.group(2))
    pixel_x = abs(float(pixel_match.group(1)))
    pixel_y = abs(float(pixel_match.group(2)))

    min_elev = float(minmax_match.group(1))
    max_elev = float(minmax_match.group(2))

    return raster_x, raster_y, pixel_x, pixel_y, min_elev, max_elev


def parse_origin_and_extent(info_text, raster_x, raster_y, pixel_x, pixel_y):
    origin_match = re.search(
        r"Origin\s*=\s*\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)",
        info_text,
    )
    if not origin_match:
        raise ValueError("Could not parse raster origin from gdalinfo output")

    origin_x = float(origin_match.group(1))
    origin_y = float(origin_match.group(2))
    min_x = origin_x
    max_x = origin_x + (raster_x * pixel_x)
    max_y = origin_y
    min_y = origin_y - (raster_y * pixel_y)

    return min_x, min_y, max_x, max_y


def count_raster_bands(info_text):
    return len(re.findall(r"^Band\s+\d+", info_text, flags=re.MULTILINE))


def remove_flat_planes(world_element):
    includes_to_remove = []
    for include in world_element.findall("include"):
        uri = include.find("uri")
        if uri is None or uri.text is None:
            continue
        if uri.text.strip() in {"model://ground_plane", "model://asphalt_plane"}:
            includes_to_remove.append(include)
    for include in includes_to_remove:
        world_element.remove(include)


def remove_old_terrain(world_element):
    models_to_remove = []
    for model in world_element.findall("model"):
        if model.get("name") == "terrain_heightmap":
            models_to_remove.append(model)
    for model in models_to_remove:
        world_element.remove(model)


def configure_gui_camera(world_element, size_z):
    gui = world_element.find("gui")
    if gui is None:
        return

    camera = gui.find("camera")
    if camera is None:
        return

    # Keep camera well above terrain on load to avoid underground starts.
    cam_z = max(40.0, size_z * 0.8)
    pose = camera.find("pose")
    if pose is not None:
        pose.text = f"-60 0 {cam_z:.3f} 0 0.75 0"

    track = camera.find("track_visual")
    if track is not None:
        min_dist = track.find("min_dist")
        if min_dist is not None:
            min_dist.text = "18.0"
        max_dist = track.find("max_dist")
        if max_dist is not None:
            max_dist.text = "260.0"
        use_model_frame = track.find("use_model_frame")
        if use_model_frame is not None:
            use_model_frame.text = "true"
        inherit_yaw = track.find("inherit_yaw")
        if inherit_yaw is not None:
            inherit_yaw.text = "true"
        xyz = track.find("xyz")
        if xyz is not None:
            # Keep camera trailing the drone in its model frame, clearly above it.
            xyz.text = "-18 0 10"


def add_heightmap_model(world_element, png_path, albedo_path, size_x, size_y, size_z):
    normal_map = "file://media/materials/textures/flat_normal.png"

    model = ET.SubElement(world_element, "model", {"name": "terrain_heightmap"})
    ET.SubElement(model, "static").text = "true"

    link = ET.SubElement(model, "link", {"name": "terrain_link"})

    collision = ET.SubElement(link, "collision", {"name": "terrain_collision"})
    collision_geom = ET.SubElement(collision, "geometry")
    collision_hm = ET.SubElement(collision_geom, "heightmap")
    height_offset = -0.5 * size_z
    ET.SubElement(collision_hm, "uri").text = png_path
    ET.SubElement(collision_hm, "size").text = f"{size_x:.3f} {size_y:.3f} {size_z:.3f}"
    ET.SubElement(collision_hm, "pos").text = f"0 0 {height_offset:.3f}"

    visual = ET.SubElement(link, "visual", {"name": "terrain_visual"})
    visual_geom = ET.SubElement(visual, "geometry")
    visual_hm = ET.SubElement(visual_geom, "heightmap")
    ET.SubElement(visual_hm, "use_terrain_paging").text = "false"
    ET.SubElement(visual_hm, "uri").text = png_path
    ET.SubElement(visual_hm, "size").text = f"{size_x:.3f} {size_y:.3f} {size_z:.3f}"
    ET.SubElement(visual_hm, "pos").text = f"0 0 {height_offset:.3f}"

    texture = ET.SubElement(visual_hm, "texture")
    ET.SubElement(texture, "diffuse").text = f"file://{albedo_path}"
    ET.SubElement(texture, "normal").text = normal_map
    ET.SubElement(texture, "size").text = f"{max(size_x, size_y):.3f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-world", required=True)
    parser.add_argument("--terrain-tif", required=True)
    parser.add_argument("--albedo-tif")
    parser.add_argument("--heightmap-png", required=True)
    parser.add_argument("--output-world", required=True)
    parser.add_argument("--heightmap-dim", type=int, default=2049)
    parser.add_argument("--albedo-dim")
    args = parser.parse_args()

    if not os.path.isfile(args.base_world):
        raise FileNotFoundError(f"Base world not found: {args.base_world}")
    if not os.path.isfile(args.terrain_tif):
        raise FileNotFoundError(f"Terrain GeoTIFF not found: {args.terrain_tif}")

    info_text = run_cmd(["gdalinfo", "-mm", args.terrain_tif])
    raster_x, raster_y, pixel_x, pixel_y, min_elev, max_elev = parse_gdalinfo(info_text)
    min_x, min_y, max_x, max_y = parse_origin_and_extent(
        info_text, raster_x, raster_y, pixel_x, pixel_y
    )
    band_count = count_raster_bands(info_text)

    if max_elev <= min_elev:
        max_elev = min_elev + 1.0

    os.makedirs(os.path.dirname(args.heightmap_png), exist_ok=True)
    os.makedirs(os.path.dirname(args.output_world), exist_ok=True)

    target_dim = args.heightmap_dim
    if not is_pow2_plus_1(target_dim):
        target_dim = nearest_pow2_plus_1(target_dim)

    # albedo dimensions resolved after terrain extent is known
    _albedo_dim_result = parse_albedo_dim(args.albedo_dim)
    albedo_width = None  # resolved below after albedo_info_text is available
    albedo_height = None
    _albedo_dim_explicit = _albedo_dim_result if _albedo_dim_result is not _ALBEDO_NATIVE_SENTINEL else None

    temp_png = args.heightmap_png + ".raw.png"
    albedo_png = args.heightmap_png + ".albedo.png"

    run_cmd(
        [
            "gdal_translate",
            "-r",
            "cubic",
            "-ot",
            "Byte",
            "-scale",
            str(min_elev),
            str(max_elev),
            "0",
            "255",
            "-outsize",
            str(target_dim),
            str(target_dim),
            args.terrain_tif,
            temp_png,
        ]
    )

    if command_exists("convert"):
        run_cmd(
            [
                "convert",
                temp_png,
                "-alpha",
                "off",
                "-colorspace",
                "Gray",
                "-type",
                "Grayscale",
                "-depth",
                "8",
                "-blur",
                "0x0.7",
                f"PNG8:{args.heightmap_png}",
            ]
        )
        os.remove(temp_png)
    else:
        os.replace(temp_png, args.heightmap_png)

    # Build a visual albedo from an external RGB orthophoto if provided, else fallback to terrain raster.
    size_x = raster_x * pixel_x
    size_y = raster_y * pixel_y
    size_z = max(max_elev - min_elev, 1.0)

    albedo_source = args.albedo_tif if args.albedo_tif and os.path.isfile(args.albedo_tif) else args.terrain_tif
    albedo_info_text = run_cmd(["gdalinfo", albedo_source])
    albedo_band_count = count_raster_bands(albedo_info_text)

    # Resolve albedo dimensions now that we have albedo_info_text and terrain extent
    if _albedo_dim_explicit is not None:
        albedo_width, albedo_height = _albedo_dim_explicit
    else:
        native = compute_native_albedo_dim(albedo_info_text, size_x, size_y)
        if native:
            albedo_width, albedo_height = native
        else:
            albedo_width, albedo_height = raster_x, raster_y

    if albedo_source != args.terrain_tif and albedo_band_count >= 3:
        warped_albedo = albedo_png + ".warp.tif"
        run_cmd(
            [
                "gdalwarp",
                "-overwrite",
                "-r",
                "lanczos",
                "-te",
                str(min_x),
                str(min_y),
                str(max_x),
                str(max_y),
                "-ts",
                str(albedo_width),
                str(albedo_height),
                "-of",
                "GTiff",
                albedo_source,
                warped_albedo,
            ]
        )
        run_cmd(
            [
                "gdal_translate",
                "-of",
                "PNG",
                "-b",
                "1",
                "-b",
                "2",
                "-b",
                "3",
                warped_albedo,
                albedo_png,
            ]
        )
        os.remove(warped_albedo)
    elif band_count >= 3:
        run_cmd(
            [
                "gdal_translate",
                "-of",
                "PNG",
                "-outsize",
                str(albedo_width),
                str(albedo_height),
                "-b",
                "1",
                "-b",
                "2",
                "-b",
                "3",
                args.terrain_tif,
                albedo_png,
            ]
        )
    else:
        run_cmd(
            [
                "gdal_translate",
                "-of",
                "PNG",
                "-ot",
                "Byte",
                "-scale",
                str(min_elev),
                str(max_elev),
                "0",
                "255",
                "-outsize",
                str(albedo_width),
                str(albedo_height),
                args.terrain_tif,
                albedo_png,
            ]
        )

    # No ImageMagick post-processing needed for albedo:
    # gdal_translate -b 1 -b 2 -b 3 already outputs clean RGB PNG without alpha.

    tree = ET.parse(args.base_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("Could not find <world> element in base world")

    remove_flat_planes(world)
    remove_old_terrain(world)
    configure_gui_camera(world, size_z)
    add_heightmap_model(world, args.heightmap_png, albedo_png, size_x, size_y, size_z)

    ET.indent(tree, space="  ")
    tree.write(args.output_world, encoding="utf-8", xml_declaration=True)

    print(f"Terrain file      : {args.terrain_tif}")
    print(f"Albedo source     : {albedo_source}")
    print(f"Heightmap PNG     : {args.heightmap_png}")
    print(f"Terrain albedo    : {albedo_png}")
    print(f"Output world      : {args.output_world}")
    print(f"Raster size       : {raster_x} x {raster_y}")
    print(f"Heightmap dim     : {target_dim} x {target_dim} (2^n+1)")
    print(f"Albedo dim        : {albedo_width} x {albedo_height}")
    print(f"Pixel size (m)    : {pixel_x} x {pixel_y}")
    print(f"Elevation range   : {min_elev} .. {max_elev}")
    print(f"Gazebo hm size    : {size_x:.3f} {size_y:.3f} {size_z:.3f}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
