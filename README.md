# Adaptive Traffic Lights with Emergency Vehicle Preemption 

## Objectives
The principal objective of this project is the conception and realization of an **adaptive intersection control system** capable of dynamically regulating vehicular and pedestrian flows while incorporating **emergency vehicle preemption**. Specifically, the system is designed to:  
- Demonstrate the application of a **finite state machine (FSM)** paradigm for safety-critical sequencing of signal phases.  
- Implement **queue-responsive phase extensions** utilizing photoresistor-based vehicle detectors with hysteresis filtering to mitigate spurious activations.  
- Provide **pedestrian service intervals** encompassing *Walk* and *Flashing Don’t Walk (FDW)* signals with guaranteed minimum crossing times.  
- Enable **preemptive priority control** for emergency vehicles through safe truncation of conflicting phases, incorporation of clearance intervals, and restoration to adaptive operation following service.  
- Offer a didactic platform for the exploration of advanced traffic control strategies within the constraints of embedded microcontroller systems.  

---

## Project Overview
The project emulates a **two-phase intersection** comprising a **North–South (NS)** and **East–West (EW)** corridor. Each corridor is instrumented with a **light-dependent resistor (LDR) sensor** to detect vehicular presence, thereby enabling adaptive extension of green phases. Pedestrian demands are introduced via tactile pushbuttons, which are latched until service is granted.  

The FSM supervises state transitions, enforcing **minimum green**, **maximum green**, **yellow change intervals**, and **all-red clearance periods** to ensure operational safety. Upon receipt of an **emergency preemption request** (via dedicated buttons for NS and EW), the controller truncates the conflicting phase, executes clearance intervals, and assigns **exclusive right-of-way** to the emergency approach for a guaranteed service window. A recovery interval restores equilibrium before the resumption of normal adaptive operation.  

The complete architecture is implemented in **Arduino Uno** within **Tinkercad Circuits**, thereby affording replicability in both virtual and physical laboratories.  
Explore, run, and modify this project directly in **Tinkercad Circuits**:
👉 [Open in Tinkercad](https://www.tinkercad.com/things/jclR9sWg7V6-adaptivetrafficpreemption)  
---

## Components
The following components are required, all of which are available within the Tinkercad environment:  

- Arduino Uno R3 microcontroller  
- Six light-emitting diodes (LEDs) with 220 Ω resistors (traffic signals: NS Green, NS Yellow, NS Red, EW Green, EW Yellow, EW Red)  
- Two photoresistors (LDRs) with 10 kΩ resistors (vehicular detectors for NS and EW)  
- Two pedestrian pushbuttons (for NS and EW crossing requests)  
- Two emergency preemption pushbuttons (for NS and EW emergency vehicle priority)  
- Breadboard and jumper wires for circuit integration  

---

## Pin Configuration
| Function              | Arduino Pin | Notes                                                        |
|-----------------------|-------------|--------------------------------------------------------------|
| NS Green / Yellow / Red | D2 / D3 / D4 | Vehicular signalization for North–South corridor             |
| EW Green / Yellow / Red | D5 / D6 / D7 | Vehicular signalization for East–West corridor               |
| NS Detector (LDR)       | A0          | Voltage divider with 10 kΩ resistor; covered = vehicle present |
| EW Detector (LDR)       | A1          | Voltage divider with 10 kΩ resistor                          |
| Pedestrian NS Button    | D8          | Request to cross EW roadway                                  |
| Pedestrian EW Button    | D9          | Request to cross NS roadway                                  |
| Emergency NS Button     | D10         | Emergency vehicle preemption for NS                          |
| Emergency EW Button     | D11         | Emergency vehicle preemption for EW                          |

---

## Timing Parameters
The default temporal parameters are as follows (expressed in milliseconds):  

- Minimum Green: 7,000  
- Maximum Green: 22,000  
- Extension Increment: 1,500  
- Yellow Interval: 2,500  
- All-Red Clearance: 900  
- Pedestrian Walk: 5,000  
- Pedestrian FDW: 7,000 (with LED flashing at 2 Hz)  
- Preemptive Green: 10,000  
- Preemptive Recovery (All-Red): 4,000  

These parameters may be tuned in the source code to replicate alternative traffic control scenarios.  

---

## Operational Principle
1. **Normal Adaptive Operation**  
   - The NS phase is initiated with a guaranteed minimum green interval.  
   - If vehicular demand persists, the green is extended in increments until either demand ceases or the maximum green is attained.  
   - A transition through yellow and all-red intervals precedes the EW green phase, with symmetrical logic applied.  
   - Pedestrian requests, if pending, are inserted at appropriate junctures following vehicular phase termination.  

2. **Pedestrian Service**  
   - Upon activation, a *Walk* interval is followed by a *Flashing Don’t Walk (FDW)* period, with vehicular traffic held at red throughout.  

3. **Emergency Preemption**  
   - If NS preemption is requested during EW service, EW green is truncated following the minimum green, then advanced through yellow and all-red intervals before NS receives exclusive green.  
   - The converse applies for EW preemption.  
   - Following preemptive service, an all-red hold interval is enforced prior to restoration of adaptive cycling.  

4. **Safety Features**  
   - Hysteresis is employed in detector logic to prevent false triggers due to transient light fluctuations.  
   - Starvation protection ensures that no approach is indefinitely denied service, even under continuous occupancy on the opposing corridor.  

---

## Applications
This project demonstrates tangible applications in:  
- **Urban traffic engineering**, where adaptive and preemptive control strategies are integral to intelligent transportation systems.  
- **Embedded systems pedagogy**, providing a multidisciplinary exercise in FSM design, sensor integration, and safety-critical logic.  
- **Emergency response optimization**, wherein preemptive signal control mitigates delays for emergency services.  

---

## Learning Outcomes
Upon completion, learners are expected to:  
- Acquire proficiency in implementing **finite state machines** for real-time embedded applications.  
- Demonstrate competence in **sensor-actuated traffic control** with noise mitigation.  
- Articulate the theoretical and practical considerations of **emergency vehicle preemption** within traffic signal systems.  
- Evaluate system performance through telemetry logs, including phase timings, queue responsiveness, and preemption latency.  

---

## Validation Protocol
The system should be validated through the following experimental procedures:  
- **Actuation Test**: Cover/uncover the LDR to simulate vehicle presence; measure extensions.  
- **Hysteresis Test**: Expose LDRs to rapid fluctuations; confirm absence of erroneous triggering.  
- **Preemption Test**: Activate preemption buttons during conflicting greens; record truncation and latency.  
- **Pedestrian Test**: Initiate pedestrian requests; verify correct Walk and FDW timing.  
- **Starvation Guard Test**: Maintain continuous occupancy on one approach; confirm eventual service to the other.  

---

## Repository Structure
adaptive_traffic_preemption/
├─ README.md
└─ adaptive_traffic_preemption.ino
