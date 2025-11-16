%Made by Not Alex Y <3
%Copy of our internal sims used to simulate at turret
% 
% To use: Run the script the parameters may be modified to modify the system dynamics or simulation behaviors
%
%


clc
clear all
close all

%Physical system properties 
% (These are reasonable numbers, but are not collected from a sysid)

g2 = 1; %Gear reduction 
c = 5.526; %N-m-s/rad Viscous damping coefficent                                                        - we need to change this potentially (torque const * no load current / no load speed) (run at (diff) const speed, measure current/torque to maintain that const speed, plot motor torque vs angular velo, that line should be damping coeff)
J = 0.04145; %kg * m^2 Mass moment of inertia                                                          - we need to def change this - would the CAD be a good estimate? no - send varying curr/torque to motor, get angular accel, plot torque vs angular accel to get your moi
kb = .763 * g2; % V-s/rad Back EMF constant. Current set to a 3508
kt = .741 * g2; % N-m/A Torque Constant Currently set to a 3508
ra = 8.705; %Ohm, Effective Armature resistance. Currently set to a 3508
la = .00578 * 2; %H effective armature inductance
uk = .741; %N-m Coulumb friction gain. NOT A COF, this is a raw torque
gearrat = 1; %Gear ratio

%Sensor/discretization parameters
Tpc = 1/500; %s Time period on IMU measurements                                                         - change this potentially
lat = .004; %s Round-trip latency                                                                        - characterize this (send an impulse power of 16000 and see how many prints till the motor has nonzero velocity)


%Controller properties


%Target parameters
thetatarget = deg2rad(170); %rad Target angle
thetadotinput = 0;

squiggle = 0; %Whether to sinusoidal target inputinput or not
squigglefreq = 3;


testvel = 0; %Whether or not to ingore position command to tune just velo
veltar = 10; %Velocity target amplitude                                                             - do we need to change

%Feedback/feedforward parameters

%Physical system limits
voltmax = 24; %Battery voltage
vmax = voltmax /(kb * gearrat); % rad/s Maximum rpm the profile generator will try to target

%Profile generator
kp = 23; % 1/s kp                                                                                   - why
thetadotbreak =  6; %When to switch controller modes                                                - what does this mean
ascale = .9; %Percentage of maximum possible accelleration to actually use
adecel = .4 * voltmax * kt * gearrat/(J * ra); %Target deceleration at high velocities

%Gain scheduling
kdt = 0; %-.5; %-1;%-.47; %Unitless, how much to modify kp by depending on drivetrain frequency     - idk how this works bro
kdtrev = 0;% -.5;% -1.3; %-.7; %Unitless kdt but when dt is spinning the other way                  - dt is drivetrain so this is gain for drivetrain in reverse



%Feedforwards parameters. Do not modify these, they are automatically modified with the system paramters
ks = uk /(kt*gearrat); %A Static feedforwards
kv = kb*gearrat; %V-s/rad Back-emf feedforwards, should be kb
ka = J/(kt*gearrat); %A-s^2/rad   Accelleration feedforwards, should be J/kt
kvis = c/(kt*gearrat); %A-s/rad Viscous damping feedforwards

%Velocity feedback parameters
kpv = .01; %.7; %1; %A-s/rad  kpv
kiv = 50; %30; %15; %A/rad  kiv
ivmax = .8 / kiv; %Maximum integral buildup. Dividing by kiv normalizes it to units of output
intthresh = voltmax*.65; %When the controller thinks it is saturated
takeback = .1; %How much percentage of the integral to take back when overshot per cycle. Used to eliminate annoying zero behavior

%Disturbance parameters, used to add disturbances to the simulation
vdt = 0; %Drivetrain velo, rad/s
damp = .4; % %Amplitude of disturbances
dfreq = 3.7; %Frequency of disturbances rad/s
namp = 0.0000000001; %Noise amplitude [0.0000003]

%Initial conditions
thetadotic = 0;
theta0 = deg2rad(0);


%Simulation parameters

tf = .5; %Maximum simulation time
minstep = .0001; %Time step for fixed-step time solvers. 
% Adaptive solvers have convergence issues due discontinuities


sim('rmturretmodelrelease.slx')
%% Plots

figure(1)

thetatargetplot = zeros(length(t), 1) + rad2deg(thetatarget);
veltargetplot = zeros(length(tvel), 1) + tvel;
mvelplot = zeros(length(mvel), 1) + mvel;
thetadeg = theta * 360 / (2*pi);

plot(t, thetadeg, t, thetadot * 30 / pi, tdis, tangle * 180 / pi, tdis, va * 100/voltmax, tdis, mangle*360 / (2*pi), tdis, manglepred *180 / pi)

legend("Angle", "RPM", "Target Angle", "Motor Duty Cycle", "Measured angle")
xlabel('T (s)')
ylabel('Degrees, RPM, Duty Cycle')
axis([0 tf -100 360])

figure(2)
plot(t, thetadot, tdis, veltargetplot, tdis, mvelplot, tdis, mvelpred)
legend("Angular velocity", "Target Velocity", "Measured Velocity", "Extrapolator")
xlabel('T (s)')
ylabel('thetadot (rad/s)')
title('Velocity')
axis([0 tf -30  30])

figure(3);

plot(t, ia)
legend("Current")
xlabel('T (s)')
ylabel('Current (A)')
axis([0 tf -10 10])

figure(4)
plot(tdis, buildup, tdis, va / 100)
legend("Integral buildup", "Applied voltage / 100")
xlabel('T (s)')
ylabel('Integral buildup')
title('Integral behavior')

figure(5)
plot(tdis, amodmax, tdis, negamodmax, t, thetadotdot)
legend("Maximum positive accelleration", "Maximum negative acceleration", "Actual acceleration")
xlabel('T (s)')
ylabel('Accel (rad/s^2)')
title('Acceleration')

velophasespace = 0;
posphasespace = 0;
i = 1;
dp = .001;
while velophasespace <= 20
    veloplot(i) = velophasespace;
    posplot(i) = posphasespace;
    volts = voltmax + velophasespace * kb;
    forces = volts/ra * kt + c*velophasespace + uk;
    accel = forces / J;
    
    posphasespace = posphasespace + dp;
    velophasespace = velophasespace + accel * dp;
    

    i = i + 1;
end

figure(6)
plot(theta, thetadot, mangle, mvel, manglepred, mvel)
xlabel('Angle (rad)')
ylabel('Velo (rad/s)')
axis([0, 2*pi, -20, 20])
title("Phase plane")
legend('Real system', 'Measured system', 'Sussy predictor')

figure(7)
plot3(t, theta, thetadot, tdis, tangle, thetadotinput, tdis, mangle, mvel, tdis, manglepred, mvelpred)
axis([0 tf 0 2*pi -30 30])
ylabel('Angle (rad)')
zlabel('Velo (rad/s)')
xlabel('Time (s)')
title('3d phase plane')
legend('Real state', 'Target state', 'measured', 'extrapolated')