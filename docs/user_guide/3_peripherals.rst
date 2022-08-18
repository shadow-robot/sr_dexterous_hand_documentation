Using Peripherals
==================

PST Sensor
----------
These are simple sensors, fitted as standard, which measure the air pressure within a bubble at
the finger tip. When the finger tip presses on an object, the pressure in the bubble increases.
The sensor incorporates an automatic drift and temperature compensation algorithm
(essentially a high pass filter with an extremely low cut off frequency).


BioTacs
-------
If the hand has optoforce sensors installed, the following topics are available. They update at 100 Hz with data from the biotac sensors, which comprises their pressure,
temperature and electrode resistance. For further information about the biotacts, refer to their `documentation <https://www.syntouchinc.com/wp-content/uploads/2016/12/BioTac_SP_Product_Manual.pdf>`_.

Topics
^^^^^^

* This topic is published by the driver at 100 Hz with data from tactile sensors:

  .. code-block::

     /rh/tactile

  Example topic message when using BioTac fingertip sensors:

  .. code-block::

          tactiles:
          -
          pac0: 2048
          pac1: 2054
          pdc: 2533
          tac: 2029
          tdc: 2556
          electrodes: [2622, 3155, 2525, 3062, 2992, 2511, 3083, 137, 2623, 2552, 2928, 3249, 2705, 3037, 3020, 2405, 3049, 948, 2458, 2592, 3276, 3237, 3244, 3119]
          -
          pac0: 0
          pac1: 0
          pdc: -9784
          tac: 32518
          tdc: 0
          electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          -
          pac0: 0
          pac1: 0
          pdc: -9784
          tac: 32518
          tdc: 0
          electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          -
          pac0: 0
          pac1: 0
          pdc: -9784
          tac: 32518
          tdc: 0
          electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          -
          pac0: 0
          pac1: 0
          pdc: -9784
          tac: 32518
          tdc: 0
          electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

* The following topics are specific for each sensor and update at 100 Hz with data from the biotac sensors, which comprises their pressure,
  temperature and electrode resistance. This topic is published from the */biotac_republisher* node which receives this
  data from the driver via the */rh/tactile* topic.

  .. code-block::

     /rh/biotac_

  Example */rh/biotac_*** topic message:

  .. code-block::

     pac0: 2056
     pac1: 2043
     pdc: 2543
     tac: 2020
     tdc: 2454
     electrodes: [2512, 3062, 2404, 2960, 2902, 2382, 2984, 138, 2532, 2422, 2809, 3167, 2579, 2950, 2928, 2269, 2966, 981, 2374, 2532, 3199, 3152, 3155, 3033]

Optoforce
----------

If the hand has optoforce sensors installed, it is recommended to use the one liner to install the docker container using the “-o true” option. Doing this, everything will be set up automatically.

For more information on setup and getting started with the optoforce sensors, `look here <https://github.com/shadow-robot/optoforce/tree/indigo-devel/optoforce>`_.

Topics
^^^^^^^

Optoforce sensor data will be published on the following topics:

.. code-block::

   /rh/optoforce_**
