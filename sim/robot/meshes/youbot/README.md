# youBot meshes (GPLv3)

These OBJ files are derived from the KUKA youBot meshes in
[youbot/youbot_description](https://github.com/youbot/youbot_description)
(commit `dbc8bc9cf61b50f4d9f485b665c843dd96f95226`), which are licensed under the
GNU General Public License v3 (see `package.xml` of that repository). The full licence text
is in [`LICENSE`](LICENSE). They are distributed here under the same licence.

Changes made by `tools/convert_youbot_meshes.py`:

- Collada (`.dae`) converted to OBJ, split into one file per material colour
- vertices moved into the URDF link frame (the URDF `<visual><origin>` offset applied)
- decimated to at most 4000 faces per file
- `arm2` and `arm3` stretched along their length (middle section only) to the longer links
  of `youbot_lift.xml` (0.155 → 0.23 m, 0.135 → 0.20 m)

This notice covers the files in this directory.
