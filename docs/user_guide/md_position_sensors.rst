Position Sensors
=================

The angle of each joint in the finger is measured by a Hall effect sensor moving through the
magnetic field of either a diametrically polarised ring magnet, or an axially polarised button
magnet. An ADC in the proximal phalange samples the two sensors for joints 1, 2 and 3, while
an ADC in the palm samples the sensor for joint 4.

The angle of each joint in the thumb is measured by a Hall effect sensor moving through the
magnetic field of either a diametrically polarised ring magnet, or an axially polarised button
magnet. An ADC in the middle phalange samples the distal two sensors, while an ADC in the
palm samples the rest.

A pair of Hall effect sensors are used to measure Joint 5. They are placed 90º apart, and the
sampled values from each sensor are added together at the host to improve the signal to noise
ratio.
