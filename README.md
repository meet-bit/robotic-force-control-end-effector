# Adjustable Force Control End Effector  

This repository contains the firmware code for an adjustable force control end effector. The project demonstrates a safe and efficient approach to gripping objects of varying sizes and weights while preventing damage or injury during operation.  

## Overview  

The force control end effector is a cable-driven robotic gripper designed to ensure safe and precise interactions in human-robot systems. Its primary feature is the implementation of closed-loop current control, which dynamically regulates the gripping force based on user input and sensor feedback.  

## Key Features  

### Robust Mechanical Design  
- Built with aluminum plates and steel cables for durability and consistent performance under heavy loads.  
- Ergonomic gripper design driven by a linear actuator for precise force transmission.  

### Firmware Capabilities  
- Implements a Proportional-Integrator (PI) control loop on MSP 430 to regulate motor currents dynamically.  
- Utilizes ADC channels to map analog inputs from a force control knob and load cell into actionable digital values.  
- Includes safety features like a trigger switch and actuator feedback to maintain operation within predefined limits.  

### Graphical User Interface (GUI)  
- A GUI displays real-time feedback on force, current, and load.  
- Allows users to adjust gripping force with precision.  

### Testing Highlights  
- Successfully gripped fragile objects (e.g., a Coke can).  
- Lifted a 25-pound dumbbell.  
- Safely held a human finger without discomfort at low current limits.  

## Results  

The end effector achieved its goals, demonstrating the importance of current control for precision and safety in robotic systems. Notable challenges included limited backdrivability in the mechanical setup, with recommendations for improving actuator performance in future iterations.  

## Applications  

This project contributes valuable insights for:  
- Human-robot interaction systems.  
- Adaptive robotic gripping mechanisms.  
- Consumer robotics requiring safe operation in proximity to humans.  

> **Note**: The GUI for real-time feedback and control is the one provided by Phidget, which comes along with the Load Cell.  

## Future Work  

- Improve backdrivability in the mechanical setup.  
- Explore advanced control algorithms for enhanced gripping precision.  
- Integrate additional sensors for improved feedback and safety.
- Refactor the firmware into a modular file structure.
- Include debouncing control on the firmware to offset tthe noise and improve accuracy.
