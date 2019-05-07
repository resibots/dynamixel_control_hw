Dynamixel control hardware's documentation
============================

dynamixel_control_hw
------------
`dynamixel_control_hw <https://github.com/resibots/dynamixel_control_hw>`__ is a C++11 library for the `Dynamixel <http://en.robotis.com/index/product.php?cate_code=101010>`__ actuators. Its goals are to be fast and to work with most Dynamixels, including the Dynamixel Pro actuators.

This library comes with a command line utility that can, among other things, be very convenient for the configuration of your actuators.

Features
--------

* supports Dynamixel series AX, MX, EX, XL and Pro
* supports all instructions and messages of both version 1 and 2 of the Dynamixel communication protocol
* full C++11 api

  * template-based API for fast and low-level access
  * class-based unified API abstracting away the differences between the actuators (as long as they use the same Dynamixel protocol).

Contents
--------

.. toctree::
  :hidden:
  :caption: dynamixel_control_hw

.. toctree::
  :maxdepth: 2

  self


Indices and tables
------------------

* :ref:`genindex`
* :ref:`search`

License and authors
-------------------

This work is distributed under the terms of the `CeCILL-C <http://www.cecill.info/licences.en.html>`_ license. © UPMC and INRIA.

The authors for this library are Dorian Goepp, Eloise Dalin, Pierre Desreumaux.
