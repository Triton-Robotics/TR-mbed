%% Load your ramp response
filename = 'response_readings/inf_yaw_ramp.txt';
raw = readmatrix(filename);

u = raw(:,1);                   % power (input)
vel = raw(:,2) * 2*pi/60;       % rpm -> rad/s
pos = raw(:,3);                 % angle (rad)
Ts = 0.001;

% Create iddata with BOTH outputs
data = iddata([pos vel], u, Ts);

%% Identify 3-state continuous-time model (recommended)
Order = 3;
sys_ss = n4sid(data, Order, 'InputName','power', ...
                          'OutputName',{'pos','vel'});

sysd = d2c(sys_ss);     % convert discrete to continuous
ss_est = ss(sysd);

A = ss_est.A;
B = ss_est.B;
C = ss_est.C;
D = ss_est.D;

assignin('base','A',A);
assignin('base','B',B);
assignin('base','C',C);
assignin('base','D',D);
assignin('base','Ts',Ts);


%% Build KF

nx = size(A,1);
ny = size(C,1);

Qn = 1e-6 * eye(nx);     % process noise
Rn = 1e-4 * eye(ny);     % measurement noise

% G = identity (process noise on all states)
G = eye(nx);

% steady-state estimator gain
[L,P] = lqe(A, G, C, Qn, Rn);

assignin('base','L',L);

    