# Mobile robot base — specifications

Values measured from the composed, scaled robot (`lizard.usd`) in Isaac Sim 5.1 (PhysX + USD world bounds). 4-inch-wheel differential-drive base with a welded Intel RealSense D435i.

## Robot base
| Property | Value |
|---|---|
| Base mass | **4.5 kg** |
| Total robot mass (base + 4 wheels) | **5.164 kg** |
| Dimensions (L × W × H) | **300 × 382 × 173 mm** (0.30 × 0.38 × 0.17 m) |
| Height span above floor | ~25 mm → ~198 mm |
| Center of mass (base-link frame) | **(−1.2, 0.5, 30.9) mm** — laterally centered, ~31 mm above the base-link origin (PhysX auto-computed) |

## Wheels
| Wheel | Count | Role | Mass (each) | Diameter | Width |
|---|---|---|---|---|---|
| `_9055` (rear) | 2 | differential drive | **0.132 kg** | **101.6 mm** (= 4.00 in) | 26.4 mm |
| `_6466` (front) | 2 | dual-omni passive caster | **0.20 kg** | 104.8 mm | 30.7 mm |

## Track & wheelbase
| Property | Value |
|---|---|
| Rear (drive) track — centre-to-centre | **357 mm** (0.357 m) |
| Front track | 352 mm |
| Wheelbase (rear → front) | **237 mm** |

## Notes
- Drive type: 2-wheel differential drive on the **rear** `_9055` wheels; front `_6466` dual-omni wheels are passive casters.
- Geometric drive track = **0.357 m**; the controller's *effective* turn track was calibrated to ≈0.56 m to account for skid during in-place pivots.
- Per-link masses confirmed by PhysX: `[4.5, 0.132, 0.132, 0.2, 0.2] kg`.
- Source: `query_specs.py` (raw output in `specs_out.txt`).
