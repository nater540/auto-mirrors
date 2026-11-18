"""The aligner of `align-core` against an independent ray-traced model of the flying optics, and a viewer of the run.

- `optics`: rays, planes, reflection, and a mirror mount tilted by its screws, exactly.
- `machine`: the machine as built: tube, both mirrors, the travels, the sensor, the screws.
- `link`: runs `align-sim external` against a machine and records every move and measurement.
- `view`: the run, frame by frame.
- `cli`: `beamviz`, which `just beamviz` runs.
"""
