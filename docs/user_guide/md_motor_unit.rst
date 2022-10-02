Motor unit
===========

In a motor hand, every degree of freedom is driven from the array of twenty motors mounted
on the forearm frame. Each motor drives two tendons to give a pull/pull control.

Force sensing is integrated into the tendons at the motors, and is used to provide compliant
movements. Each pair of tendons couples a motor to a joint.

Each motor is managed by the Hand PC and completely controlled by the on-board electronics.

Small vs Large
--------------

Two types of motors are used in this hand design.

**Small Motors**: Sixteen small motors are used for all finger joints, and most thumb joints.

+----------------------------------+---------------+--------+
| Part                             | Value         | Units  |
+==================================+===============+========+
| Motor                            | Maxon 118608  |        |
+----------------------------------+---------------+--------+
| Gear                             | Maxon 352367  |        |
+----------------------------------+---------------+--------+
| Motor power                      | 3             | W      |
+----------------------------------+---------------+--------+
| Gear ratio                       | 131:1         |        |
+----------------------------------+---------------+--------+
| Max continuous safe tendon load  | 65            | N      |
+----------------------------------+---------------+--------+
| Tendon Load at full power        | 110           | N      |
+----------------------------------+---------------+--------+
| Max continuous current           | 217           | mA     |
+----------------------------------+---------------+--------+

**Large Motors**: Four large motors are used for the two wrist joints, and thumb joints 4 and 5.

+----------------------------------+---------------+--------+
| Part                             | Value         | Units  |
+==================================+===============+========+
| Motor                            | Maxon 110151  |        |
+----------------------------------+---------------+--------+
| Gear                             | Maxon 143988  |        |
+----------------------------------+---------------+--------+
| Motor power                      | 6             | W      |
+----------------------------------+---------------+--------+
| Gear ratio                       | 128:1         |        |
+----------------------------------+---------------+--------+
| Max continuous safe tendon load  | 190           | N      |
+----------------------------------+---------------+--------+
| Tendon Load at full power        | 190           | N      |
+----------------------------------+---------------+--------+
| Max continuous current           | 350           | mA     |
+----------------------------------+---------------+--------+

Tensioner
----------
Each motor unit (except for the wrist) includes a tensioner unit. These exist purely to maintain
a little tension on each tendon at all times, so that the tendons wind tidily onto the motor
spool. The tension applied by the tensioner is very slight, and does nothing to compensate for
backlash.

Torque measurement
-------------------
The tendons exit the motor spool, and take a 90º turn around a metal bar. This bar is held at
each end by a load cell, which is set up to measure the vertical component of any load exerted
on the bar. The torque is calculated as the difference between the two readings.

If the tendons are not correctly adjusted, the tensioners may not be able to maintain tension on
the spool. In this case, the tendons can become tangled around the spool.

Slack adjustment and tensioning.
--------------------------------
Refer to the section: **Maintaining the hand**.

The spool is split into two halves which can rotate relative to each other, but are normally held
by a bolt fixedly with respect to each other. To take up any tendon slack, simply loosen the bolt,
rotate the top half of the spool, and re-tighten the bolt.

Only one tendon is tight at any one time, while the other tendon is fairly loose. However, the
loose tendon is kept under very slight tension by the tensioner to help it wind around the spool
tidily. The wrist motors do not have tensioners.