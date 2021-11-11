# Note

This is a fork of Autoware.Auto containing modifications to support usage with the [CARMAPlatform](https://github.com/usdot-fhwa-stol/carma-platform). This repository contains changes to the Autoware source code and configuration that may not be supported by the Autoware Foundation and may not be consistent with the original design intent of Autoware. All modifications in this repository are licensed under the same Apache License 2.0 as Autoware and all modifications of the source code made will be marked as such in accordance with the terms of the Apache License 2.0. For a list of modifications and their descriptions please see [NOTICE.md](NOTICE.md).

## For developers working in this repository

For any modified file please follow these steps to ensure proper documentation of this modification in compliance with the terms of the Apache License 2.0:

1. Add a comment at the top of any modified file with a high-level description of the modification and date the modification was made.
2. Add a high-level description and date of the overall modification to the [NOTICE.md](NOTICE.md) file.

## Modifications

- Addition of notice about fork status to the README.md and creation of this NOTICE.md file
  - 11/11/2021
  - Michael McConnell

- COLCON_IGNORE files added to lgsvl_interface, ne_raptor_interface, spinnaker_camera_driver, spinnaker_camera_nodes, and xsens_driver as these packages are not required by CARMA Platform
  - 11/11/2021
  - Michael McConnell

- autoware_set_compile_options_reduced_warning function created in autoware_auto_cmake to reduce error warning levels in come packages where external deps were tripping error checks.
  - 11/11/2021
  - Michael McConnell
- Multiple packages updated to use autoware_set_compile_options_reduced_warning allowing them to build in carma-base docker images.
  - 11/11/2021
  - Michael McConnell
