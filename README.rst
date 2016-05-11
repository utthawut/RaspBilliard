RaspBillard
====================
The RaspBilliard is Low-cost Virtual Realistic Billiard Simulator, 
running on Raspberry Pi

Acknowledgements
====================
Development of this project would not have been possible without following document and library. 

1.  AN EVENT-BASED POOL PHYSICS SIMULATOR http://web.stanford.edu/group/billiards/AnEventBasedPoolPhysicsSimulator.pdf  

2.  POOL PHYSICS SIMULATION BY EVENT PREDICTION II: COLLISIONS http://web.stanford.edu/group/billiards/PoolPhysicsSimulationByEventPrediction2Collisions.pdf  

3.  pi3d <https://github.com/tipam/pi3d>  


Dependencies
====================
- pi3d
- numpy
- cython

Concept Overview
====================
The arcade machine/simulator will consist of accelerometer and infrared sensor to measure,

- How fast is the cue ball collided with the cue stick (m/s)
- The Angle that the cue stick is elevated (0-90 degress)
- The position of impact on the cue ball (Cue-Ball impact)




.. image:: images/game_1.jpg
   :align: right
   
.. image:: images/game_concept_1.jpg
   :align: right
   
.. image:: images/game_concept_2.jpg
   :align: right
   
.. image:: images/game_concept_3.jpg
   :align: right
   
.. image:: images/game_concept_4.jpg
   :align: right
   
.. image:: images/schemetic.jpg
   :align: right
   
Thank you Mr. Theeraphan Upan, he help me write this schemetic. And he will make PCB soon.

[RaspBilliard video](https://www.youtube.com/watch?v=04EXz3RdWic)
