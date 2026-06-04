#!/usr/bin/env python3
"""make_abs_urdf_h2.py — Rewrite relative mesh paths to absolute for Isaac Sim.

Isaac Sim's URDF importer does not resolve relative paths, so all mesh
references must be absolute filesystem paths.

The H2 URDF uses relative paths:  ../meshes/<name>.STL
The meshes live at:               ~/topstar_h2/h2_model/meshes/

Usage (paths are optional — defaults use ~/topstar_h2/h2_model/):
  python3 make_abs_urdf_h2.py [INPUT_URDF [OUTPUT_URDF]]
"""
import os
import sys
import xml.etree.ElementTree as ET

H2_MODEL_DIR = os.path.expanduser("~/topstar_h2/h2_model")
URDF_DIR = os.path.join(H2_MODEL_DIR, "urdf")

input_urdf  = sys.argv[1] if len(sys.argv) > 1 else os.path.join(URDF_DIR, "h2.urdf")
output_urdf = sys.argv[2] if len(sys.argv) > 2 else os.path.join(URDF_DIR, "h2_abs.urdf")

urdf_dir = os.path.dirname(os.path.abspath(input_urdf))

ET.register_namespace("", "")
tree = ET.parse(input_urdf)
root = tree.getroot()

count = 0
for mesh in root.iter("mesh"):
    fn = mesh.get("filename", "")
    if fn and not os.path.isabs(fn):
        abs_path = os.path.normpath(os.path.join(urdf_dir, fn))
        mesh.set("filename", abs_path)
        count += 1

tree.write(output_urdf, xml_declaration=True, encoding="utf-8")
print(f"Wrote {output_urdf} ({count} mesh paths made absolute)")
