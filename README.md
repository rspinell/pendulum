# pendulum
450 Real-Time Embedded Systems Project Background

The purpose of this project is to analyze, design, and develop a micro controller platform to perform real time control stabilizing a 
non-linear fan pendulum system. For this control problem, a pendulum was defined as a rigid massless beam, where there is a point of mass 
located at the free end of the beam. A DC motor acted as both a point mass in the classical pendulum control problem, as well as the 
source of the control force. To simplify the problem, the pivoting end of the beam was constrained to only allow for the pendulum to 
rotate in one plane of motion. The control problem can be defined as commanding the system to an angular position 𝜃, in which the motor and 
propellor assembly applies thrust to establish a new equilibrium at the specified 𝜃.

The motor is a hobby king brushless DC motor, and the driving ESC, is a 6𝐴 Plush Turnigy ESC. The ESC requires 5𝑉 as a signal high input, 
ground and the servo control signal, also 7.4𝑉 as drive power to the motor. Both the position sensor of the system and user input device 
are 10𝐾Ω potentiometers, where the position sensor is secured to the fulcrum of the pendulum.

The control system was implemented with an STM32L476 ultra-low-power microcontroller, which features several ADC configurable channels and 
sufficient 16-bit PWM timers. For this project, we utilized two ADC channels to read voltage measurements from and one PWM output to drive 
the servo controller. The 2𝑉 output of the PWM signal was sufficient in driving a standard ESC. The STM32L4 5𝑉 output channel is employed 
as a reference high to power the ESC and sensor array of two potentiometers. The potentiometer fixed to fulcrum of the pendulum is defined
as the System potentiometer, where the Reference potentiometer, is used as a user input or voltage reference to allow for implementation of
real-time input variances and ultimately, real-time system response.

For the external servo controller (ESC), the duty cycle needed to be scaled appropriately to accurately drive the ESC. The ESC received 
packets, maximum 2𝑚𝑠 in length. These packets need to be formatted: 0 – 1𝑚𝑠 high, 1 – 2𝑚𝑠, varies the throttle position. To get zero 
throttle 1𝑚𝑠 pulses were sent, and full throttle at 2𝑚𝑠 pulses. Our PWM duty cycle then scaled back to 350 𝐻𝑧 so that we had enough time 
between counter turn overs to send these pulses and then have some time left over as a buffer. In this way, we avoided operating on the 
ragged edge of what the ESC could handle, while still maintaining adequate control of the system (with a fast-enough update speed).

The feedback loop consists of two potentiometers and ADC channel blocks. The potentiometers describe the angular position of the pendulum 
beam as a varying resistance directly proportional to the change in theta of the pendulum, where the reference voltage is the position the 
user commands the pendulum to. The microcontroller continually reads both voltage measurements after it has been processed through an 
integrated Analog-to-Digital Convertor (ADC). The error from the desired and measured thetas is passed to a PID controller block, which 
produces gains that are subsequently added to the steady state duty cycle. The duty cycle is then converted to the corresponding PWM 
waveform and scaled to the aforementioned ESC protocol. Passing the conditioned PWM signal through to the ESC, the signal is converted to a 
three-phase drive signal to drive the DC brushless motor. The DC brushless motor provides a torque to the propeller, which provides thrust 
normal to the pendulum. 

To obtain the desired control response, a steady state duty cycle output, proportional, and determined by the mathematical model to the 
requested position, was implemented:

𝑇h𝑟𝑢𝑠𝑡(𝑆𝑡𝑒𝑎𝑑𝑦 𝑆𝑡𝑎𝑡𝑒 𝑂𝑢𝑡𝑝𝑢𝑡 𝑃𝑊𝑀) ∗ 𝑙𝑒𝑛𝑔𝑡h = 𝑚𝑎𝑠𝑠 ∗ 𝑔𝑟𝑎𝑣𝑖𝑡𝑦 ∗ 𝑙𝑒𝑛𝑔𝑡h ∗ sin(𝑡h𝑒𝑡𝑎 𝑟𝑒𝑓𝑒𝑟𝑒𝑛𝑐𝑒)

The steady state output occupied a certain bandwidth of duty cycle, which allowed for the remaining bandwidth to be used to control the 
system. In addition to the steady state response, a PID controller was added with the remaining duty cycle. Initially, a conventional PID 
was employed, which used the voltage of the reference theta minus the voltage of the system theta as the error term. After iterative 
controller tuning and testing, we chose to use a sinusoidal term, in place of the standard linear P term in our PID controller. This was 
determined to effectively account for the nonlinear nature of system response. Both “I” and “D” terms were included, using one present, 
and past state to determine the integral and derivative. This showed massive improvements over the original completely linear PID. An 
anti-integrator wind up condition was implemented to improve steady state response, and increase the stability of the system.


