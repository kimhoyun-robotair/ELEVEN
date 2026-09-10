# Asset provenance and reuse

All architectural geometry, furniture, elevator parts, button labels, surface
textures and rendered images in this repository were generated specifically
for this project in Blender. No external 3D asset, photo, HDRI, font file or
texture library is redistributed. Text geometry uses Blender's bundled Bfont
and is converted into mesh by export.

The contributors dedicate their rights in these original generated assets to
the public domain under **CC0 1.0 Universal**. Commercial use, modification,
simulation, dataset generation and redistribution are permitted. The full
CC0 legal instrument is at <https://creativecommons.org/publicdomain/zero/1.0/legalcode>.

| Asset | Source | Terms |
|---|---|---|
| Office and residence architecture | `tools/build_worlds.py` | CC0-1.0 |
| Furniture and household details | `tools/furniture.py` | CC0-1.0 |
| Opaque and glass elevator cars, shafts, panels | `tools/build_worlds.py` | CC0-1.0 |
| Base color, roughness and normal PNG textures | Seeded procedural texture generator in `tools/build_worlds.py` | CC0-1.0 |
| `.blend`, `.gltf`, `.bin`, `.usdc`, `.usda` scene content and render stills | Generated from the above sources | CC0-1.0 |
| Python and JavaScript project code | This repository | MIT (`LICENSE`) |
| Three.js 0.180.0 and its loader/controls/environment helpers | [Three.js source](https://github.com/mrdoob/three.js/tree/r180) | MIT; verbatim notice in `web/dist/vendor/THREE-LICENSE` |

Blender itself, Isaac Sim and OpenUSD are build/runtime tools, not bundled
assets. Their separate software licenses continue to apply. No NVIDIA asset
pack or Isaac binary is included. The web demo vendors the exact Three.js
modules it needs and makes no third-party CDN requests.
