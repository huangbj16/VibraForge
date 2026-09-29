# Mechanical Design

This folder contains CAD files for control units and vibration units. In our lab environment, parts are printed with PLA materials using 0.4mm nozzles on [Bambu Lab X1C 3D Printer](store.bambulab.com/collections/3d-printer/products/x1-carbon). Other printers with higher precision should also satisfy the requirements.

- Control_Unit_Large
- Control_Unit_Small — two enclosure variants with the same 49 × 47 mm footprint. Print each enclosure together with its matching cap.
  - **With battery** (`Enclosure_with_Battery.stl`, `Cap_with_Battery.stl`): the original design, with space for a battery. The enclosure is 22 mm tall and the cap 8.5 mm. STL only. These files were previously named `Enclosure.stl` and `Cap.stl`.
  - **Battery-free** (`Enclosure_Battery_Free.stl`, `Cap_Battery_Free.stl`): a newer, slimmer design for a control unit powered over USB with no battery. The enclosure is 8.5 mm shallower (13.5 mm tall) and the cap is 11.5 mm tall. Editable STEP files are included.
- Vibration_Unit_LRAX
- Vibration_Unit_LRAZ
- Vibration_Unit_VCA