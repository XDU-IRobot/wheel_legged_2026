# Wheel-legged offline tools

The controller-generation files are organized by responsibility:

```text
tools/
├─ wheel_legged_model.py          Shared five-DOF physical model
├─ lqr/
│  ├─ generate_scheduled_kctrlp.py
│  ├─ generated/kctrlp_generated.hpp
│  ├─ reports/kctrlp_fit_report_*.html
│  └─ legacy/HerKules_VOCAL_SJ_LQR_v4_with_data.m
└─ leso/
   ├─ generate_scheduled_leso.py
   ├─ generated/leso_model_generated.hpp
   └─ reports/leso_model_fit_report.html
```

`wheel_legged_model.py` is the single source of truth for the model equation,
robot-specific physical parameters, leg tables, state transformation, and the
construction of `M`, `G`, `B_tau`, `A`, and `B`.

## Regenerate LQR data

Run from `tools/lqr`:

```powershell
python generate_scheduled_kctrlp.py --output --html-report --report-variant all
```

Omitting `--output` and `--html-report` performs a dry run.

## Regenerate LESO data

Run from `tools/leso`:

```powershell
python generate_scheduled_leso.py --output --html-report
```

Omitting `--output` and `--html-report` performs a dry run.

The generated LESO data follows:

```text
M * q_ddot + G * q = B_tau * u + d
```

The runtime observer must use the previous cycle's final four-dimensional
virtual command after mode overrides and virtual-channel saturation.
