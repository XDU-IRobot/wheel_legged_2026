# Wheel-legged offline tools

The controller-generation files are organized by responsibility:

```text
tools/
|-- wheel_legged_model.py          Shared five-DOF physical model
|-- cad/
|   `-- collect_leg_mass_properties.cs
|-- lqr/
|   |-- generate_scheduled_kctrlp.py
|   |-- generated/kctrlp_generated.hpp
|   |-- reports/kctrlp_fit_report_*.html
|   `-- legacy/HerKules_VOCAL_SJ_LQR_v4_with_data.m
`-- leso/
    |-- generate_scheduled_leso.py
    |-- generated/leso_model_generated.hpp
    `-- reports/leso_model_fit_report.html
```

`wheel_legged_model.py` is the single source of truth for the model equation,
robot-specific physical parameters, leg tables, state transformation, and the
construction of `M`, `G`, `B_tau`, `A`, and `B`.

## CAD leg model

The current single-leg CAD mass is `1.640907 kg`. The three robot profiles
temporarily use the same independently copied leg table so they can be replaced
one by one later.

Each table row is:

```text
[l0, c_parallel_from_upper, c_normal, I_pitch_at_COM]
```

`c_parallel_from_upper` is measured along the virtual leg from its upper pivot
towards the wheel axle. `c_normal` is the signed in-plane offset from that
line. The LQR and LESO generators both consume the same reduction:

```text
l_b = c_parallel_from_upper
l_w = l0 - c_parallel_from_upper
I_reduced = I_pitch_at_COM + m_leg * c_normal^2
```

This is the upright linearization of the two-dimensional offset-COM model and
reduces exactly to the old collinear model when `c_normal = 0`. The normal
offset also creates an affine (constant at zero leg angle) gravity torque.
That constant is deliberately left in the LESO disturbance channel instead of
being inserted into the linear `G*q` term; doing the latter would incorrectly
turn a bias into stiffness.

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
