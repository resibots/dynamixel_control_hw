.. _download_and_compilation:

Download and Compilation
========================

Download
--------

To get **dynamixel_control_hw**, simply clone the source code from https://github.com/resibots/dynamixel_contrl_hw.git with git.

.. note:: Don't forget to clone it into your catkin_workspace directory ``cd $HOME/catkin_ws/src`` .

Dependencies
------------

- `ROS kinetic <http://wiki.ros.org/kinetic>`__
- python2.x
- C++
- `libdynamixel <https://github.com/resibots/libdynamixel>`__
- ROS Control::

    apt-get install ros-kinetic-ros-control

.. note:: You need to store the installation path of `libdynamixel <https://github.com/resibots/libdynamixel>`__ in your environment variable inside your  ``~/.bashrc`` file  ``export LIBDYNAMIXEL=/home/USER/Resibots`` .


ROS Dependencies :

- catkin
- rospy
- roscpp
- urdf
- std_msgs
- message_generation
- hardware_interface
- combined_robot_hw
- controller_manager
- ros-controllers
- joint-state-publisher::

    apt-get install ros-kinetic-catkin ros-kinetic-rospy ros-kinetic-roscpp ros-kinetic-urdf ros-kinetic-std-msgs ros-kinetic-message-generation ros-kinetic-hardware-interface ros-kinetic-combined-robot-hw ros-kinetic-controller-manager ros-kinetic-ros-controllers ros-kinetic-joint-state-publisher



Building
--------

.. highlight:: shell

The build system for this package is **catkin** . Don't run away yet. Here is how we compile and install it :

.. note:: don't forget to install `ROS kinetic <http://wiki.ros.org/kinetic>`__ before


1. Clone
    simply clone the package `dynamixel_control_hw <https://github.com/resibots/dynamixel_control_hw>`__ in our catkin workspace source folder ::

      cd $HOME/catkin_ws/src
      git clone https://github.com/resibots/dynamixel_control_hw.git

2. Compilation
    go into our workspace directory, then compile ::

      cd ..
      catkin_make


Building the documentation
--------------------------

.. note::
    This section is only useful for developers who need to update the documentation.

Install sphinx via pip: ::

    sudo pip install Sphinx
    sudo pip install sphinxcontrib-bibtex

.. warning::

  On Mac OSX, do not use `brew install sphinx` because this is not the right sphinx

Install the Resibots theme for Sphinx::

    git clone https://github.com/resibots/sphinx_resibots_theme
    export SPHINX_RESIBOTS_THEME="/home/me/path/to/sphinx_resibots_theme"

Install `breathe <https://breathe.readthedocs.io/en/latest/>`_ via pip::

    sudo pip install breathe

Install `doxygen <http://www.stack.nl/~dimitri/doxygen/>`_ via your package manager (e.g. apt-get / brew)::

    apt-get install doxygen

create a temporary folder where we generate the documentation::

    rm -rf /tmp/doc_dynamixel_control_hw
    mkdir /tmp/doc_dynamixel_control_hw
    cd /tmp/doc_dynamixel_control_hw
    git clone git@github.com:resibots/dynamixel_control_hw.git
    cd dynamixel_control_hw/
    git checkout doc
    cd doc
    make html
    git checkout gh-pages
    cd ..
    cp -r doc/_build/html/* .
    git add .
    git commit -m 'manualy update of the doc [skip ci]'
    git push origin gh-pages
    rm -rf /tmp/doc_dynamixel_control_hw

.. note:: in general, we're using the branch **doc** to write the documentation
          and the branch **gh-pages** which contains html files.

About sphinx and ReStructuredText:
  - `Primer for ReStructuredText <http://sphinx-doc.org/rest.html>`_, the markup language of Sphinx,
  - `markup specific to Sphinx <http://sphinx-doc.org/markup/index.html>`_,
  - `About C++ in Sphinx <http://sphinx-doc.org/domains.html#id2>`_
  - `Breathe (bridge between sphinx and doxygen) <https://breathe.readthedocs.org/en/latest/>`_
