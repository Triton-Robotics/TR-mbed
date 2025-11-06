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

% Doesn't work, need to fix or make sense of it

U = fft(input);
Ym = fft(response);

H_est = Ym ./ U;

tf_est = idfrd(H_est, F, Ts);

bode(H_est);

sys_tf = tfest(tf_est, 2, 1);
[b, a] = tfdata(sys_tf, 'v');


%% Find State Space response then Convert to Transfer Fn

% Best method for consistently good model + PID values

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

final_tf = tf(b, a, dt_ms);

opts = pidtuneOptions('DesignFocus', 'reference-tracking');
crossover = 100; % target crossover freq is 1/10 the hz
[C_PID, info] = pidtune(final_tf, "PID", crossover);

Kp = C_PID.Kp;
Kd = C_PID.Kd;
Ki = C_PID.Ki;

%info about the tuning
info


%% Testing bs

margins = allmargin(final_tf);


%% Lead-Lag

alpha = 0.1;
w_c = 4;
tau = 1/(sqrt(alpha)*w_c);
K = 1;     % initial guess

lead = K * (tau*s + 1)/(alpha*tau*s + 1);

beta = 10;
tau_l = 1;  % pick lag corner lower than crossover
lag = (beta*tau_l*s + 1)/(tau_l*s + 1);

C_leadlag = lead * lag;


%% Systune using our SLX file

mdl = "leadlag";
open_system(mdl)

st = slTuner(mdl,"vel_leadlag");

% use SYSTUNE ITS SO GOOD

addPoint(st, "vel_out");
addPoint(st, "vel_in");
addPoint(st, "out");

% figure out tuninggoals
req3 = TuningGoal.Margins('vel_out',margins.GainMargin,60);
req4 = TuningGoal.Overshoot('vel_in','vel_out',20);

rng(0);
TunedST = systune(st,[req3,req4]);

showTunable(TunedST)
C = getBlockValue(TunedST, 'vel_leadlag');
tf(C)
refresh(st)


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