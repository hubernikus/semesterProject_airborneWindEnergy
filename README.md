# Semester Project - Agile Flight Control
This project is created to control and visualize an aircraft in agile flight is simulation. It is part of mechanical engineering master studies at EPFL.

@EPFL @LA @AigleFlightControl @SemsterProject @AirborneWindEnergy

--

### Libraries:
Following main function are given:
#### motionSimulation
Allows for different simulation and control type.
At the beginning of the document following parameters can be changed to adapt simulation:
- motionType
linear / circular / MPC_simu / screw
- vsiual (for 2D or 3D simulation)
2 / 3
- control
'None', 'LQR', 'steadyStatePredction'
- linearSystemDim (Dimension of the linearized system for the LQR controller - with 6: vel, angRate; 9: vel, angRate, pos; 10: vel, angRate, quat; 13: vel, angRate, pos, quat)
6 / 9 / 10 / 13
simuName (name of the simulation, choosing 'None' and the simulation is not saved to file)


#### equilibriumSearch_steadyStateMotion
This allows for the equilibrium search of the different steady state motion types. With corresponding commands:
- steadyLevel_longitudinal
- longitudinalFlight
- stedyState_circle
- steadyState_cirle (screw drive motion)


Following base functions:
### Libraries:
Libraries contains following scripts:
#### controllib.py
The control library includes functions needed for control, such as:
- saturateControl (checks, whether the control is not in the range and reports it)
- saturateControl (which limits the control to maximum range)
- checkControlability_lin (checks controllability)
- checkStabilizability_lin (checks stabilazibility)
- lqr (LQR controller)
- dlqrt (discrete LQR controller)


#### nonLinContr_lib.py
The linear control library contains the mathematical tools, such as:
- LieDerivative (evaluates the Lie derivative of any order n>=0)
- LieBracket (evaluaates the Lie bracket for any order n>=0)

Further more there is a testing script to evaluate the function based on Nonlinear
Control (J.J. Slotine)


#### quatlib.py
The quaternion library contains libraries to handle quaternions:
- quatinv (inverse of an quaternions)
- quatmul (quaternion mutliplication)
- quatrot (quaternion rotation)
- quatrot_inv (inverse quaternion rotation)
- quat2_eul_rotationRate (rotation rate in quaternion)
- quat2eul (conversion from quaternion to Euler angles)
- eul2quat (conversion from Euler angles to quaternions)
- slerp (Spherical linear interpolation with quaternion to create smooth quaternion trajectories)


#### visual_lib.py
The visualization library helps to set up and visualize in 2D or 3D mode. Such as:
- initFigure_2d (initialize 2d figure)
- initFigure_3d (initialize 3d figure)
- setAxis_3d (set the 3d axis)
- drawPlane3D (draws an airplane model with defined color and opacity)
- draw_posPred (draw predicted postion for predicting-LQR controller)
- draw_aimingPosition (draw the position and the path to which the predictied-LQR controller is stabilizing to)


--
## Set up
The program was developed in Python 3.5.2

CasAdi 3.3.0 is needed for execution
https://github.com/casadi/casadi/wiki

Furthermore following directories are needed:
- model (contains directory with the airplane model *.xml file)
- ../steadyState_modes (steady state modes are saved an loaded from this directory)
- ../fig (used for automated figure output; can also be deactivated)
- ../simulationResults (automated printing simulation to files; can be deactivated)# Semester Project - Agile Flight Control
This project is created to control and visualize an aircraft in agile flight is simulation. It is part of mechanical engineering master studies at EPFL.

@EPFL @LA @AigleFlightControl @SemsterProject @AirborneWindEnergy

--

### Libraries:
Following main function are given:
#### motionSimulation
Allows for different simulation and control type.
At the beginning of the document following parameters can be changed to adapt simulation:
- motionType
linear / circular / MPC_simu / screw
- vsiual (for 2D or 3D simulation)
2 / 3
- control
'None', 'LQR', 'steadyStatePredction'
- linearSystemDim (Dimension of the linearized system for the LQR controller - with 6: vel, angRate; 9: vel, angRate, pos; 10: vel, angRate, quat; 13: vel, angRate, pos, quat)
6 / 9 / 10 / 13
simuName (name of the simulation, choosing 'None' and the simulation is not saved to file)


#### equilibriumSearch_steadyStateMotion
This allows for the equilibrium search of the different steady state motion types. With corresponding commands:
- steadyLevel_longitudinal
- longitudinalFlight
- stedyState_circle
- steadyState_cirle (screw drive motion)


Following base functions:
### Libraries:
Libraries contains following scripts:
#### controllib.py
The control library includes functions needed for control, such as:
- saturateControl (checks, whether the control is not in the range and reports it)
- saturateControl (which limits the control to maximum range)
- checkControlability_lin (checks controllability)
- checkStabilizability_lin (checks stabilazibility)
- lqr (LQR controller)
- dlqrt (discrete LQR controller)


#### nonLinContr_lib.py
The linear control library contains the mathematical tools, such as:
- LieDerivative (evaluates the Lie derivative of any order n>=0)
- LieBracket (evaluaates the Lie bracket for any order n>=0)

Further more there is a testing script to evaluate the function based on Nonlinear
Control (J.J. Slotine)


#### quatlib.py
The quaternion library contains libraries to handle quaternions:
- quatinv (inverse of an quaternions)
- quatmul (quaternion mutliplication)
- quatrot (quaternion rotation)
- quatrot_inv (inverse quaternion rotation)
- quat2_eul_rotationRate (rotation rate in quaternion)
- quat2eul (conversion from quaternion to Euler angles)
- eul2quat (conversion from Euler angles to quaternions)
- slerp (Spherical linear interpolation with quaternion to create smooth quaternion trajectories)


#### visual_lib.py
The visualization library helps to set up and visualize in 2D or 3D mode. Such as:
- initFigure_2d (initialize 2d figure)
- initFigure_3d (initialize 3d figure)
- setAxis_3d (set the 3d axis)
- drawPlane3D (draws an airplane model with defined color and opacity)
- draw_posPred (draw predicted postion for predicting-LQR controller)
- draw_aimingPosition (draw the position and the path to which the predictied-LQR controller is stabilizing to)


--
## Set up
The program was developed in Python 3.5.2

CasAdi 3.3.0 is needed for execution
https://github.com/casadi/casadi/wiki

Furthermore following directories are needed:
- model (contains directory with the airplane model *.xml file)
- ../steadyState_modes (steady state modes are saved an loaded from this directory)
- ../fig (used for automated figure output; can also be deactivated)
- ../simulationResults (automated printing simulation to files; can be deactivated)
