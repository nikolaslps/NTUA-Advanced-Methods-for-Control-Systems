N_steps = 100;      % Number of time steps
N_particles = 1000; % Number of particles

% Initialize particles
particles = randn(1, N_particles);              % Initial state particles from Gaussian
weights = ones(1, N_particles) / N_particles;   % Uniform initial weights
x_PF = zeros(1, N_steps);                       % Particle filter state estimates

%% This code is the same as in data_generation.m
% True state and measurement initialization
x_true = zeros(1, N_steps);     % True state
y = zeros(1, N_steps);          % Measurements

% Generate true state and measurements
for k = 2:N_steps
    w_k = randn * w_std; % Process noise
    v_k = randn * v_std; % Measurement noise
    x_true(k) = 0.5 * x_true(k - 1) + beta * x_true(k - 1) / (1 + x_true(k - 1) ^ 2) + 8 * cos(1.2 * k) + w_k;
    y(k) = alpha * x_true(k) + x_true(k) ^ 2 / 20 + v_k;
end
%%
% Particle filter loop
for k = 2:N_steps
    % Prediction step
    for j = 1:N_particles
        w = randn * w_std; % Process noise
        particles(j) = 0.5 * particles(j) + beta * particles(j) / (1 + particles(j) ^ 2) + 8 * cos(1.2 * k) + w;
    end

    % Correction Step
    likelihood = exp(-(y(k) - (alpha * particles + particles.^2 / 20)).^2 / (2 * v_std^2));
    weights = likelihood .* weights;
    weights = weights / sum(weights); % Normalize weights

    % Resampling Step
    % Resampling based on each particle weight in order to reduce particle
    % degeneracy
    indices = resampleParticles(weights, N_particles);
    particles = particles(indices);
    weights = ones(1, N_particles) / N_particles; % Reset weights after resampling

    % Estimate state
    x_PF(k) = sum(weights .* particles);
end

% Plot results
figure;
plot(1:N_steps, x_true, 'k--', 'LineWidth', 2); hold on;
plot(1:N_steps, x_PF, 'r', 'LineWidth', 1.5);
plot(1:N_steps, x_est, 'g', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('State');
title('True State vs Particle Filter Estimate vs EKF Estimate');
legend('True State', 'PF Estimate', 'EKF Estimate');
grid on;


