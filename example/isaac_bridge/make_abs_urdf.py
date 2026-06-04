#!/usr/bin/env python3
"""make_abs_urdf.py — Rewrite package:// mesh URIs to absolute paths.

Isaac Sim's URDF importer does not resolve ROS package:// URIs, so all mesh
references must be absolute filesystem paths.

The Topstar URDF uses:  package://Topstar/meshes/<name>.STL
The meshes live at:     <urdf_dir>/meshes/<name>.STL
  (i.e. the "Topstar" package root is the directory containing the URDF)

Usage (paths are optional — defaults resolve relative to this script):
  python3 make_abs_urdf.py [INPUT_URDF [OUTPUT_URDF]]

Defaults:
  INPUT_URDF  = <repo>/example/src/urdf/h1/Topstar.urdf
  OUTPUT_URDF = <repo>/example/src/urdf/h1/h1_abs.urdf
"""
import os
import sys
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_DIR   = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "src", "urdf", "h1"))

input_urdf  = sys.argv[1] if len(sys.argv) > 1 else os.path.join(URDF_DIR, "Topstar.urdf")
output_urdf = sys.argv[2] if len(sys.argv) > 2 else os.path.join(URDF_DIR, "h1_abs.urdf")

# The URDF directory acts as the ROS package root for "Topstar".
pkg_root = os.path.dirname(os.path.abspath(input_urdf))
pkg_prefix = "package://Topstar/"

ET.register_namespace("", "")
tree = ET.parse(input_urdf)
root = tree.getroot()

count = 0
for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn.startswith(pkg_prefix):
        rel = fn[len(pkg_prefix):]          # e.g. "meshes/base_link.STL"
        mesh.set("filename", os.path.join(pkg_root, rel))
        count += 1
    elif fn and not os.path.isabs(fn):      # bare filename fallback
        mesh.set("filename", os.path.join(pkg_root, fn))
        count += 1

tree.write(output_urdf, xml_declaration=True, encoding="utf-8")
print(f"Wrote {output_urdf} ({count} mesh paths made absolute)")
