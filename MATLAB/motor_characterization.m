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
response = response_raw(:,2)*2*pi/60; % rpm to rad/s
omega = response_raw(1,3);

% creating iddata obj so we can process it in the file
data = iddata(response, input, dt_ms);


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

final_tf = tf(b, a, dt_ms);

% plot rlocus
figure (1)
rlocus(final_tf)

%plot bode
figure(2)
bode(final_tf)


%% Generating the Hessian

function [F, G] = buildPrediction(Ad,Bd,Cd,Dd,N)
    nx = size(Ad,1);
    nu = size(Bd,2);
    ny = size(Cd,1);

    % preallocate
    F = zeros(ny*N, nx);
    G = zeros(ny*N, nu*N);

    Ad_pow = eye(nx);
    for i = 1:N
        Ad_pow = Ad_pow * Ad;               % Ad^i
        F(((i-1)*ny+1):(i*ny), :) = Cd * Ad_pow;
        for j = 1:i
            % contribution of u_{j-1} to y_i
            G_block = Cd * (Ad^(i-j)) * Bd;
            if ~isempty(Dd) && j==i
                G_block = G_block + Dd;    % direct feedthrough for same-step input
            end
            G(((i-1)*ny+1):(i*ny), ((j-1)*nu+1):(j*nu)) = G_block;
        end
    end
end

N = 1;

[F_h, G] = buildPrediction(A, B, C, D, N);




%% PID Tuning OR PUT IT INTO PID_AUTOTUNE.SLX

opts = pidtuneOptions('DesignFocus', 'reference-tracking', 'NumUnstablePoles', 2);
crossover = F / 10; % target crossover freq is 1/10 the hz
[C_PID, info] = pidtune(final_tf, "PID", crossover, opts);

Kp = C_PID.Kp;
Kd = C_PID.Kd;
Ki = C_PID.Ki;

%info about the tuning
info


%% Lead Lag

lead_num = [3.819, 1];
lead_den = [1, 1];

mdl = 'turret_system';
blk = 'vel_leadlag';

load_system(mdl);
ST0 = slTuner(mdl, blk);
addPoint(ST0, {'r','y'});

opt = looptuneOptions( ...
    'RandomStart', 5, ...            % try multiple initial guesses
    'MaxIter', 200);

[ST, fSoft, gHard] = looptune(ST0, 'r', 'y', crossover, opt);

% Extract tuned controller
C_LL = getBlockValue(ST, blk);
disp('Tuned Lead-Lag Controller:');
LL_tf = tf(C_LL)

[lead_num, lead_den] = tfdata(LL_tf, 'v');

writeBlockValue(ST)

figure(3)
bode(getIOTransfer(ST0, 'r', 'y'))
figure(4)
rlocus(getIOTransfer(ST0, 'r', 'y'))

allmargin(getIOTransfer(ST0, 'r', 'y'));


%% Control sys tuner

controlSystemTuner(mdl);

%% Visualize PID tune results

% Simulate closed-loop step response
Tcl = feedback(C_PID*final_tf, step_amp);      % unity-feedback
t = 0:dt_ms:10;                   % choose simulation window
figure(7)
step(Tcl, t);
grid on;
title('Closed-loop step response with pidtune result');

[Gm,Pm,Wcg] = margin(C_PID*final_tf);   % loop margins
disp([Gm, Pm, Wcg]);


%% Plot Root Locus and Bode Plot for Tuned System

% Open-loop transfer function (Controller * Plant)
L = C_PID * final_tf;
T = feedback(L, 1);

% Root Locus of tuned closed-loop system
figure(5);
rlocus(T);
title('Root Locus of Tuned Open-Loop System');

% Bode plot of closed-loop system
figure(6);
bode(T);
grid on;
title('Bode Plot of Closed-Loop System');


%% END