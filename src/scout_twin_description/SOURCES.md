# Sources and reconstruction evidence

Checked 2026-09-12. Dimensions are SI in `config/robot.json`. The photographic reconstruction is a dimension-constrained, manually interpreted model; the photographs do not establish calibrated camera extrinsics, hidden geometry, material BRDFs, payload mass or inertia.

## Agile-X SCOUT MINI

Primary source: the user's `SCOUT_MINI_USER_MANUAL.pdf`, section 1.2 (PDF pages 8-9), section 5.1 (PDF page 45), section 5.2 (PDF page 46). Page numbers here count the cover as PDF page 1.

| Quantity | Source value | Interpretation |
|---|---:|---|
| Overall length / width / height | 612 / 580 / 245 mm | Section 1.2 and dimension drawing |
| Wheelbase | 451 mm | Fore-aft axle-center separation in side view, PDF p.45 |
| Track | 490 mm | Left-right wheel-center separation in front view, PDF p.45 |
| Tire diameter | 160 mm | Direct dimension in front view, PDF p.45; ordinary treaded tire, not mecanum rollers |
| Tire width | 90 mm | Derived from overall width minus track: 580 - 490 |
| Ground clearance | 115 mm | Section 1.2; not the axle height |
| Kerb mass | 23 kg | Bare rover; excludes photographed additions |
| Maximum speed | 3 m/s | Section 1.2; no loaded-tower stability validation implied |
| Turning radius | 0 mm | Section 1.2; independent four-wheel drive and differential steering |
| Motors | 4 x 250 W DC brushless | Section 1.2; the separate 150 W note concerns mecanum variant |
| Maximum payload | 10 kg | Safety section, PDF p.3 |
| Deck rails | 350 mm long; 230 mm center spacing; 16 mm high | Section 5.2, PDF p.46 |

The table's English labels “Axle Track” and “Front/rear track” are ambiguous. The actual engineering drawing resolves them to wheelbase = 451 mm and transverse track = 490 mm. Radius is consequently 0.080 m. Suspension is visible in photographs but modelled as fixed cosmetic components; it is not a calibrated suspension model.

## Livox MID-360

Official [product page](https://www.livoxtech.com/mid-360) and [MID-360 User Manual v1.2, April 2024](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf).

| Quantity | Manufacturer specification |
|---|---:|
| Width / depth / height | 65 / 65 / 60 mm |
| Mass | Approximately 265 g |
| Horizontal / vertical FOV | 360 degrees / -7 to +52 degrees |
| Near blind zone | 0.1 m |
| Range at 100 klx | 40 m at 10% reflectivity; 70 m at 80% |
| Point rate / typical frame rate | 200,000 points/s / 10 Hz |
| Embedded IMU | ICM40609; accelerometer and gyroscope; 200 Hz |
| IMU location in LiDAR point-cloud coordinates | x = +11.00, y = +23.29, z = -44.12 mm; parallel axes |
| LiDAR point-cloud origin | On central axis, 47.0 mm above housing bottom |
| Dome start / overall height | 39.5 / 60.0 mm above housing bottom |

Manual printed pp.9, 15, 19 and 20 (PDF pp.11, 17, 21 and 22) support these values. Relative to the housing-bottom center, the IMU is consequently (11.00, 23.29, 2.88) mm when axes match the vendor point-cloud convention. The scan is non-repetitive; a geometric approximation cannot reproduce its exact temporal pattern, reflectivity response or firmware. Simulated IMU has no extra visible housing.

## Orbbec Gemini 336L

Primary source: official [Gemini 336L product specifications](https://www.orbbec.com/products/stereo-vision-camera/gemini-336l/), model G40155-180.

| Quantity | Manufacturer specification |
|---|---:|
| Width / height / depth | 124 / 29 / 27.7 mm |
| Mass | 135 g |
| Depth FOV | Horizontal 90 degrees, vertical 65 degrees |
| RGB FOV | Horizontal 94 degrees, vertical 68 degrees |
| Maximum depth mode | 1280 x 800 at 30 fps |
| Maximum RGB mode | 1280 x 800 at 60 fps |
| Depth range | 0.17-20 m+; optimal 0.25-6 m |
| Mounting | Bottom 1/4-20 UNC; back 2 x M4 |

The packaged render resolution/rate can be lower for performance and must not be interpreted as the manufacturer's maximum mode. Pinhole intrinsics derived from nominal FOV are simulation intrinsics, not a calibrated physical camera. No stereo matcher, IR illumination, lens distortion or vendor noise model is inferred from the photographs.

## Photographic interpretation and remaining measurements

The user's ten photographs were inspected. `1000025967.jpg` identifies forward as the smiling panel / MID-360 / front bumper side. Close-ups `1000025976.jpg` and `1000025977.jpg` establish two lower RGBD cameras facing opposite sides; the elevated third camera faces forward, perpendicular to the lower pair. All three camera bars are horizontal. `1000025974.jpg` establishes the MID-360 on a black platform immediately above the forward deck, with the hemispherical optical window above a finned metal base.

User-stated structure height: approximately 1.05 m. The model interprets this as aluminium profile height above the deck, excluding the camera brackets; this is an explicit interpretation, not a confirmed total robot height. Profile cross-section, frame footprint, plates, bolt placement, camera bracket heights, battery/computer enclosures, payload inertia and sensor translations are photo estimates. Exact measurements and extrinsic calibration should replace these estimates before quantitative sim-to-real use. Cables are intentionally omitted as requested.

The yellow face panel is recreated from the photograph as a visual feature. Small bevels, smooth curved surfaces, tread detail and physically based materials improve appearance but do not turn an uncalibrated photo reconstruction into surveyed manufacturer CAD.
