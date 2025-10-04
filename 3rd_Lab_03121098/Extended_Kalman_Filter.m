% Initial Conditions
x_true = zeros(1, N_steps);  % True state
y_meas = zeros(1, N_steps);  % Measurements
x_est = zeros(1, N_steps);   % EKF state estimate
P_est= zeros(1, N_steps);
P_est(1) = 1;                % Initial estimate covariance
x_est(1) = 0;                % Initial state estimate

% Measurements and EKF
for k = 1:N_steps
    % True state
    x_true(k + 1) = 0.5 * x_true(k) + beta * x_true(k) / (1 + x_true(k) ^ 2) + 8 * cos(1.2 * k) + w;
    
    % Measurement 
    y_meas(k) = alpha * x_true(k) + x_true(k) ^ 2 / 20 + v;
end

% EKF
for k = 1:N_steps-1
    
    % Prediction Step
    x_pred = 0.5 * x_est(k) + beta * x_est(k) / (1 + x_est(k) ^ 2) + 8 * cos(1.2 * k);
    F = 0.5 * beta * (1 + x_est(k) ^ 2) / (1 + x_est(k) ^ 2) ^ 2; % Jacobian of f(x)
    Q = w_std ^ 2;
    P_pred = F * P_est(k) * F' + Q; 
    
    % Update Step
    H = alpha + x_pred / 10; % Jacobian of h(x)
    R = v_std ^ 2;
    y_pred = alpha * x_pred + x_pred ^ 2 / 20;
    K = P_pred * H / (H * P_pred * H' + R);
    x_est(k + 1) = x_pred + K * (y_meas(k + 1) - y_pred);
    P_est(k + 1) = (1 - K * H) * P_pred;
    
end

% Truncate as did in data_generation script
x_true = x_true(1:end-1);
%x_est = x_est(1:end-1);
P_est = P_est(1:end-1);

figure;
for k = [4, 6, 7]
    subplot(1, 3, find([4, 6, 7] == k));
    X_min = -35;
    X_max = 35;
    x = linspace(X_min, X_max, N_steps);

    % Compute the probability density 
    x_true_distribution = normpdf(x, x_true(k), w_std);
    y_meas_likelihood = normpdf(x, alpha * x_true(k) + x_true(k) ^ 2 / 20, v_std);
    x_est_distribution = normpdf(x, x_est(k), sqrt(P_est(k)));
    
    plot(x, x_true_distribution, 'g-', 'LineWidth', 2); hold on;
    plot(x, y_meas_likelihood, 'r', 'LineWidth', 1);
    plot(x, x_est_distribution, 'b--', 'LineWidth', 1.5);
    hold off;
    
    legend('True State', 'Measurement  Likelihood', 'EKF Estimate');
    xlabel('State');
    ylabel('Probability Density');
    title('Extended Kalman Filter (EKF) Performance');
    grid on;
    
end