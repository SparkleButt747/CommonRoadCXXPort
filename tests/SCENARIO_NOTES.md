# Single-track scenario consistency tests

The files in this directory exercise the Python and C++ implementations of the
single-track (ST) vehicle model with identical input traces.  The shared input
command log lives in [`tests/data/single_track_trace.csv`](data/single_track_trace.csv)
and can be regenerated with [`tests/data/generate_single_track_trace.py`](data/generate_single_track_trace.py).

Running `python tests/test_scenario_consistency.py --build-dir <cmake-build>`
performs the following steps:

1. Integrate each trace with the Python model (`python_scenario_runner.py`)
   using a shared RK4 integrator and record the commanded steering rate,
   longitudinal acceleration, and resulting state/derivative histories.
2. Run the C++ companion executable (`scenario_simulator`) on the same trace to
   generate matching CSV output.
3. Compare the two outputs with absolute tolerances of `1e-12` for the command
   channels and `5e-8` for state and derivative histories.  These thresholds
   accommodate minor floating-point drift while still flagging meaningful
   regressions.

Any future changes that alter the tolerances or input definitions should update
this note so downstream maintainers understand the expected alignment between
implementations.
