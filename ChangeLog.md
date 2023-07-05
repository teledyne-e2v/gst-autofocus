=== Version 1.1 ===

06-05-2022

- Tweak some parameters to improve the autofocus speed
- The sharpness calculation is now normalised by the average of the pixels squared to be less sensitive to light level change
- Improvement to the autofocus algorithms to make them more reliable

=== Version 1.2 ===

20-05-2022

- Fix to the two phase algorithm
- Added the option to recalibrate the autofocus using the black pattern of the sensor
- Added a new parameter to hold a pda value multiple frames
- Added more debugging information

=== Version 1.3 ===

05-07-2023

- New autofocus algorithms
- Possibility to control the sharpness calculation
- Compile only with Meson, autotools suppresed
