% We want A, B, C, D to be in the workspace from the prev file/ function,
% then we run root_locus.m AND input these values into pid_autotune.slx

syms s


%% Load data from .txt
filename = 'pluh.txt';
response_raw = readmatrix(filename);
% Parameters
step_amp = 8191;  % imaginary power number, we should switch this with a real value
dt_ms = 0.015; % we might get to change this FINALLY


%% Step Response
input = response_raw(:,1);
idx = input == step_amp;
response = response_raw(idx,2);

% creating iddata obj so we can process it in the file
data = iddata(response, input, dt_ms);


%% Direct transfer function estimation

sys_tf  = tfest(data, 2, 1); % 2 poles 1 zero

[numerat, denominat] = tfdata(sys_tf,'v'); 

% Change numerator and denominator to "b" and "a" to test this


%% Find State Space response then Convert to Transfer Fn

sys_ss = n4sid(data, 2); % second order discrete ss
ss_est = ss(sys_ss);

A = ss_est.A;
B = ss_est.B;
C = ss_est.C;
D = ss_est.D;

% State Space to Transfer Function

[b, a] = ss2tf(A, B, C, D);

% Convert b and a into numerator and denominator respectively
numerator = 0;
for i = 0:(len(b)-1)
    numerator = numerator + b(len(b) - i) * s^i;
end

denominator = 0;
for i = 0:(len(a)-1)
    denominator = denominator + a(len(a) - i) * s^i;
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

final_tf = tf(num, den, dt_ms);

crossover = 0; % if we wanna add it ourselves
[C, info] = pidtune(final_tf, "PID");

Kp = C.Kp;
Kd = C.Kd;
Ki = C.Ki;

%info about the tuning
info

%% Visualize PID tune results

% Simulate closed-loop step response
Tcl = feedback(C*final_tf, step_amp);      % unity-feedback
t = 0:dt_ms:10;                   % choose simulation window
step(Tcl, t);
grid on;
title('Closed-loop step response with pidtune result');

[Gm,Pm,Wcg] = margin(C*final_tf);   % loop margins
disp([Gm, Pm, Wcg]);


%% END