clc; clear; close all;

%% ---------------------------------------------------------------
%  Control Input Dataset
%
%  v(k)   : Linear velocity              [m/s]
%  om(k)  : Angular velocity             [rad/s]
%  t(k)   : Discrete time sequence       [s]
%
%  The control input vector is defined as:
%      u(k) = [v(k); om(k)]
% ---------------------------------------------------------------
load("data\my_input.mat");

%% ---------------------------------------------------------------
%  Measurement Dataset
%
%  r(k)   : range measurement              [m]
%  b(k)   : bearing measurement            [rad]
%  l(k)   : position of the 6 landmarks in X,Y coordinates   [m]
%
%  The Measurement vector is defined as:
%      z(k) = [r(k); b(k)]
% ---------------------------------------------------------------
load("data\my_measurements.mat");

%% ==============================================================
%  Data Sanity Check
% ===============================================================
fprintf('r range: min=%.3f max=%.3f mean=%.3f\n', min(r(:),[],'omitnan'), max(r(:),[],'omitnan'), mean(r(:),'omitnan'));
fprintf('b range: min=%.3f max=%.3f mean=%.3f\n', min(b(:),[],'omitnan'), max(b(:),[],'omitnan'), mean(b(:),'omitnan'));
fprintf('max |b| = %.3f\n', max(abs(b(:)),[],'omitnan'));

%% ==============================================================
%  EKF INITIALISATION
% ===============================================================

% ----- State -----
x_pos = 0.5;    % [m]
y_pos = 1;      % [m]
theta = 0;      % [rad]
x = [x_pos; y_pos; theta];

% ----- lidar distance respect to robot centre -----
d = 0; % [m]

% ----- Timestep -----
dt = 1;       % [s]

% ----- Number of control steps -----
N_u = length(v);

% ----- Number of state -----
N_x = N_u + 1;

% ----- Process Noise Covariance Matrix (Q) -----
std_lin_vel = 0.007;     % [m/s]
std_ang_vel = 0.0075;    % [rad/s]
Q = diag([std_lin_vel^2, std_ang_vel^2]);

% ----- Measurement Noise Covariance Matrix (R) -----
std_range = 0.02;        %[m]
std_bearing = 0.15;      %[rad]
R = diag([std_range^2, std_bearing^2]);

% ----- State Error Covariance Matrix (P) -----
std_x0 = 0.2;             %[m]
std_y0 = 0.2;             %[m]
std_theta0 = deg2rad(2);  %[rad]
P = diag([std_x0^2, std_y0^2, std_theta0^2]);
P_row = size(P, 1);
P_col = size(P, 2);
P_hist = nan(P_row, P_col, N_x);
P_hist(:,:,1) = P;

% ----- Cavans and Information Print  -----
x_hist = zeros(3, N_x);
x_hist(:,1) = x;

% ----- Innovation Analysis  -----
innovation = [nan; nan];
LM_row = size(l, 1);
r_inno_hist = nan(N_u, LM_row);
b_inno_hist = nan(N_u, LM_row);
mea_dimension = size(innovation, 1);
chi_confi_range = 0.95;

% ----- NIS(Normalized Innovation Square) Analysis  -----
NIS_hist = nan(N_u, LM_row);

% ----- Single or Muti Landmark  -----
single_mode_enabled = false;
if single_mode_enabled
    LM_row = 1;
else
    LM_row = size(l, 1);
end 

%% ===============================================================
%  EKF Main Loop
% ===============================================================
for k = 1:N_u % 1 - 612

    % Get control input from dataset by timestep
    v_k = v(k);
    om_k = om(k);

    % 1. State Prediction - Prior State (x_pri)
    x_pri = [
        x(1) + dt * cos(x(3)) * v_k;
        x(2) + dt * sin(x(3)) * v_k;
        wrapToPi(x(3) + dt * om_k);
    ];
    
    % 2. State Jacobian (F) - ∂fx/∂x,  ∂fx/∂y,  ∂fx/∂th;
    %                         ∂fy/∂x,  ∂fy/∂y,  ∂fy/∂th;
    %                         ∂fth/∂x, ∂fth/∂y, ∂fth/∂th;
    F = [
        1, 0, -dt * sin(x(3)) * v_k;
        0, 1, dt * cos(x(3)) * v_k;
        0, 0, 1;
    ];

    % 3. Process Noise Jacobian (L) - ∂fx/∂wv,  ∂fx/∂ww;
    %                                 ∂fy/∂wv,  ∂fy/∂ww;
    %                                 ∂fth/∂wv, ∂fth/∂ww;
    L = [
        dt * cos(x(3)), 0;
        dt * sin(x(3)), 0;
        0, dt
    ];

    % 4. Prediction Error Covariance Matrix - Prior P (P_prior)
    P_pri = F * P * F' + L * Q * L';

    % 5. Number of landmark from [my_measurement.mat] 
    % row_l = size(l, 1);
    % row_l = 1;

    % 6. Update the state for further calculation in landmark loop
    x = x_pri;
    P = P_pri;

    for j = 1:LM_row
    % for j = 1:2

        % 7. Get the position of each landmark in 2D (x, y)
        x_l = l(j, 1);
        y_l = l(j, 2);

        % 8. Predict range measurement 
        % -  formula: √((x_l − x_k − d cos θ_k)² + (y_l − y_k − d sin θ_k)²) 
        dx = x_l - x(1) - d * cos(x(3));
        dy = y_l - x(2) - d * sin(x(3));

        r_pred = sqrt(dx^2 + dy^2);

        % Sanity Check 
        % - Protect the calculation in 12. innovation Jacobian(H) since r_pred as denominator
        if r_pred < 1e-6
            continue;
        end

        % 9. Predict bearing measurement
        % -  formula: atan2(y_l − y_k − d sin θ_k , x_l − x_k − d cos θ_k) − θ_k
        b_pred = wrapToPi(atan2(dy, dx) - x(3));

        % 10. Get real measurement from lidar data [my_measurement.mat] 
        r_real = r(k, j);
        b_real = b(k, j);

        % Sanity Check
        % - Exclude nan value of range and bearing measurement in dataset
        if isnan(r_real) || isnan(b_real)
            continue;
        end
        
        % 11. Calculate innovation
        % -   formula： ν_k^l = z_k^l(real measurement) − ẑ_k^l(predict measurement)
        r_inno = r_real - r_pred;
        b_inno = wrapToPi(b_real - b_pred);
        innovation = [r_inno; b_inno];

        r_inno_hist(k, j) = r_inno;
        b_inno_hist(k, j) = b_inno;

        % 12. Innovation Jacobian (H) - ∂r/∂x, ∂r/∂y, ∂r/∂th;
        %                               ∂b/∂x, ∂b/∂y, ∂b/∂th; 
        H = [
            -dx/r_pred, -dy/r_pred, d/r_pred*(dx*sin(x(3)) - dy*cos(x(3)));
            dy/r_pred^2, -dx/r_pred^2, -d/r_pred^2*(dy*sin(x(3)) + dx*cos(x(3)))-1;
        ];

        % 13. Innovation Covariance Matrix
        % -   formula: S = HPH' + R 
        S = H * P * H' + R;

        % 14. Kalman Gain
        K = P * H' / S;

        % 15. Update State - post state (x)
        x = x + K * innovation;
        x(3) = wrapToPi(x(3));

        % 16 Update Prediction Error Covariance Matrix - post error matrix (P)
        % Joseph form: 𝑃 = ( 𝐼 − 𝐾 𝐻 ) 𝑃 ( 𝐼 − 𝐾 𝐻 ) 𝑇 + 𝐾 𝑅 𝐾 𝑇 
        % Simplified form:𝑃 = ( 𝐼 − 𝐾 𝐻 ) 𝑃  
        % P = (eye(3) - K * H) * P;
        I = eye(3);
        P = (I - K * H) * P * (I - K * H)' + K * R * K';

        % 17. NIS (Normalized Innovation Squared)
        % formula NIS​ = z(innovation)'/​S(innovation covariance)*​z(innovation)​
        NIS = innovation' / S * innovation;
        NIS_hist(k, j) = NIS;

    end

    % Update Print Information of state (x)
    x_hist(:,k+1) = x;
    % Update Print Information of error (P)
    P_hist(:,:,k+1) = P;

end

% draw animation of car moving
animate_ekf(x_hist, P_hist, l, t);

% Consistency Evaluation by innovation of range and bearing measurement
analysis_innovation(r_inno_hist, b_inno_hist, t);

% Consistency Evaluation by NIS (Normalized Innovation Squared)

analysis_NIS(NIS_hist, mea_dimension, chi_confi_range);

% Stability Evaluation by Prediction Error Covariance Matrix (P)
analysis_P(P_hist, t);

