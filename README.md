# kuka_interface_packages
This repo includes packages needed to control the KUKA Robots with a modular interface. (i.e. as a seperate "KUKA" bridge mode) in ROS with robot-toolkit.

###Dependencies
- [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit) (from epfl-lasa)
- [motion-generators](https://github.com/epfl-lasa/motion-generators) (from epfl-lasa)

---
###Funcionality

This package offer a bridge control interface for the KUKA robot using robot-toolkit. This way, instead of having your controllers implemented within the lwr-interface framework (as most robot-toolkit examples) we have a modular architecture, where control policies and motion planners can be implemented in a seperate node and the ```kuka_fri_bridge``` deals with setting the control modes and bi-directional communication with the robot. 

It has two types of control modes.

  1. Joint Impedance Control Mode:
![alt tag](https://cloud.githubusercontent.com/assets/761512/10713622/224bc630-7ac1-11e5-96cd-ef2b83aa87cb.png)
  
  2. Cartesian Impedance Control Mode:
![alt tag](https://cloud.githubusercontent.com/assets/761512/10713605/6167fbfa-7ac0-11e5-95c9-523ffbbf7db5.png)

