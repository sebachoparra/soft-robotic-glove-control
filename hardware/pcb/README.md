# PCB Design

This directory holds the PCB design files for the soft glove's electronics
(RP2040/Pico carrier board, sensor breakout, pneumatic driver board, etc.).

Suggested layout as designs are added:

```
hardware/pcb/
├── <board-name>/
│   ├── <board-name>.kicad_pro    (or .sch / .brd for other EDA tools)
│   ├── <board-name>.kicad_sch
│   ├── <board-name>.kicad_pcb
│   ├── gerbers/                  # fabrication output (zip or individual files)
│   ├── bom.csv                   # bill of materials
│   └── README.md                 # board purpose, revision notes, pinout
```

Keep exported fabrication outputs (Gerbers, drill files, BOM, pick-and-place)
alongside the source project files so a board revision is fully
reproducible from the repo alone.

Large binary CAD libraries or 3D step models, if added later, should go
through Git LFS rather than being committed directly — ask before adding
anything over a few MB.
