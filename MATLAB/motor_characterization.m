% We want A, B, C, D to be in the workspace from the prev file/ function,
% then we run root_locus.m AND input these values into pid_autotune.slx

syms s


%% Load data from .txt
filename = 'response_readings/inf_yaw_ramp.txt';
% response_raw = readtable(filename);
% response_raw = table2array(response_raw);
response_raw = readmatrix(filename);
% [rows, columns] = size(response_raw);
    

% Parameters
step_amp = 8191;  % imaginary power number, we should switch this with a real value
dt_ms = 0.001; % sampling time
Ts = dt_ms;
F = 1 / Ts;


%% Step Response
input = response_raw(:,1);
idx = input == step_amp;
input = response_raw(idx,1);
response = response_raw(idx,2);

plot(response)

% creating iddata obj so we can process it in the file
data = iddata(response, input, Ts);


%% Other Response Types (Ramp, Sinusoidal, White noise + Amp)
input = response_raw(:,1);
response = response_raw(:,2);
omega = response_raw(1,3);

% creating iddata obj so we can process it in the file
data = iddata(response, input, dt_ms);


%% DFT for function

U = fft(input);
Ym = fft(response);
pluh = "hi";

H_est = Ym ./ U;

tf_est = idfrd(H_est, omega, Ts);

%bode(tf_est);

sys_tf = tfest(tf_est, 2, 1);
[b, a] = tfdata(sys_tf, 'v');


%% Direct transfer function estimation

sys_tf  = tfest(data, 2, 1); % 2 poles 1 zero

[b, a] = tfdata(sys_tf,'v'); 

% Change numerator and denominator to "b" and "a" to test this


%% Find State Space response then Convert to Transfer Fn

% tbh direct tf is better

sys_ss = n4sid(data, 2); % second order discrete ss
ss_est = ss(sys_ss);

A = ss_est.A;
B = ss_est.B;
C = ss_est.C;
D = ss_est.D;

% State Space to Transfer Function

[b, a] = ss2tf(A, B, C, D);


%% Convert b and a into numerator and denominator respectively
numerator = 0;
for i = 1:(length(b))
    numerator = numerator + b(i) * s^(length(b)-i);
end

denominator = 0;
for i = 1:(length(a))
    denominator = denominator + a(i) * s^(length(a)-i);
end


%% Root locus of transfer function

function [G] = root_locus(num, denom)
    sym_tf = (num/denom);

    % Simplify to a single rational expression
    [num_sym, den_sym] = numden(sym_tf);
     
    % Convert symbolic polynomials to numeric coefficient vectors
    num_coeffs = sym2poly(num_sym);
    den_coeffs = sym2poly(den_sym);
     
    % Create the transfer function G using the numeric coefficient vectors
    G = tf(num_coeffs, den_coeffs);
    
    % plot rlocus
    figure (1)
    rlocus(G)
    
    %plot bode
    figure(2)
    bode(G)
end

root_locus(numerator, denominator)


%% PID Tuning OR PUT IT INTO PID_AUTOTUNE.SLX

% I can use pidtune to tune the pid here itself!!! (i wanna use
% pidautotune tho)

final_tf = tf(b, a, dt_ms);

opts = pidtuneOptions('DesignFocus', 'reference-tracking');
crossover = 100; % target crossover freq is 1/10 the hz
[C_PID, info] = pidtune(final_tf, "PID", crossover);

Kp = C_PID.Kp;
Kd = C_PID.Kd;
Ki = C_PID.Ki;

%info about the tuning
info


%% Systune using our SLX file

mdl = "pid_autotune";
open_system(mdl)

st = slTuner(mdl,"vel_PID");

% use SYSTUNE ITS SO GOOD

addPoint(st, "vel_out");
addPoint(st, "vel_in");

% figure out tuninggoals
req3 = TuningGoal.Margins('vel_out',10,80);
req4 = TuningGoal.Overshoot('vel_in','out',20);

opt = systuneOptions('RandomStart',3);
rng(0);
[st,fSoft,~,info] = systune(st,[req3,req4],opt);

showTunable(st)

% refresh(st)


%% Visualize PID tune results

% Simulate closed-loop step response
Tcl = feedback(C_PID*final_tf, step_amp);      % unity-feedback
t = 0:dt_ms:10;                   % choose simulation window
figure(3)
step(Tcl, t);
grid on;
title('Closed-loop step response with pidtune result');

[Gm,Pm,Wcg] = margin(C_PID*final_tf);   % loop margins
disp([Gm, Pm, Wcg]);


%% Plot Root Locus and Bode Plot for Tuned System

% Open-loop transfer function (Controller * Plant)
L = C_PID * final_tf;

% Closed-loop transfer function
T = feedback(L, 1);

% Root Locus of tuned open-loop system
figure(4);
rlocus(L);
title('Root Locus of Tuned Open-Loop System');

% Bode plot of tuned open-loop system
figure(5);
bode(L);
grid on;
title('Bode Plot of Tuned Open-Loop System');

% Root Locus of tuned closed-loop system
figure(6);
rlocus(T);
title('Root Locus of Tuned Open-Loop System');

% Bode plot of closed-loop system
figure(7);
bode(T);
grid on;
title('Bode Plot of Closed-Loop System');


%% END