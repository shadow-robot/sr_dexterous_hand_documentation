+----------------------------+----------------------------------------+------------------------------------------+----------------------------------------+
| PWM                        | Teach Mode                             | Position                                 | Trayectory(*)                          |
+============================+========================================+==========================================+========================================+
| Control input: PWM demand  | Control input: Torque (Effort) demand  | Control input: Position demand           | Control input: Position demand + time  |
| Input refreshment: 1kHz    | Input refreshment: 1kHz                | Input refreshment: 1kHz                  |                                        |
| Implemented in: Motor side | Control loop: 5kHz                     | Control loop: 1kHz                       | (*) Position Control with the addition |
| Control output: PWM demand | Implemented in: Motor side             | Implemented in: Host side                | of one algorithm on the top which      |                                  |                            | Control output: PWM demand             | Control output: PWM demand               | splits the position target into a      |
|                            | Sensor feedback: Motor strain gauges   | Sensor feedback: Joint position sensors  | collection of points, creating a spline|
|                            |                                        |                                          | which controls the speed of the joint  |
|                            |                                        |                                          |                                        |
|                            |                                        |                                          |                                        |
+----------------------------+----------------------------------------+------------------------------------------+----------------------------------------+
