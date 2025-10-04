% Initialization
num_particles = 1000;                  % Number of particles
num_steps = 200;                       % Number of steps in the simulation
S_particles = 0.4 * ones(num_particles, 1);  % Initial Susceptible state
E_particles = zeros(num_particles, 1);      % Initial Exposed state
I_particles = 0.01 * ones(num_particles, 1); % Initial Infected state
weights = ones(num_particles, 1) / num_particles; % Uniform weights

% Parameters
r1 = 0.1;
r2 = 0.3;
b1 = 0.005;
b2 = 0.1;
b3 = 0.05;
b4 = 0.2;
dt = 1;

% Arrays to store the averages for plotting
avg_S = zeros(num_steps, 1);
avg_E = zeros(num_steps, 1);
avg_I = zeros(num_steps, 1);

% Observation model noise
w4 = exp(randn * 0.3); 

% Simulation of the system and Particle Filter
for t = 1:num_steps
    % Observation model (measurement)
    y(t) = (0.2 * E_particles(1) + I_particles(1)) * w4; 

    % Prediction Step (Particle Update)
    for j = 1:num_particles
        % Stochastic noise factors
        w1 = exp(randn * 0.5);
        w2 = exp(randn * 0.3);
        w3 = exp(randn * 0.3);

        % Update S, E, I for each particle using the model
        S_particles(j + 1) = S_particles(j) + dt * ((-r1 * E_particles(j) * S_particles(j) - r2 * I_particles(j) * S_particles(j)) * w1 + b1 * (1 - E_particles(j) - I_particles(j) - S_particles(j)) * w2);
        E_particles(j + 1) = E_particles(j) + dt * ((r1 * S_particles(j) * E_particles(j) + r2 * I_particles(j) * S_particles(j)) * w1 - w3 * (b2 + b3) * E_particles(j));
        I_particles(j + 1) = I_particles(j) + dt * w3 * (b2 * E_particles(j) - b4 * I_particles(j));
    end

    % Correction Step (Weight Update based on Measurement)
    for j = 1:num_particles
        y_pred = (0.2 * E_particles(j) + I_particles(j)) * w4;
        % Log-normal distribution likelihood
        likelihood = (1 / (y_pred * sqrt(2 * pi * 0.3^2))) * exp(-((log(y_pred) - log(y(t)))^2 / (2 * w4^2)));
        weights(j) = weights(j) * likelihood;
    end

    % Normalize the weights
    weights = weights / sum(weights);

    % Resampling Step
    % Resampling based on particle weights to reduce degeneracy
    indices = randsample(1:num_particles, num_particles, true, weights);
    S_particles = S_particles(indices);
    E_particles = E_particles(indices);
    I_particles = I_particles(indices);

    % Store average values for plotting
    avg_S(t) = mean(S_particles);
    avg_E(t) = mean(E_particles);
    avg_I(t) = mean(I_particles);
end

% Predicted values of S, E, I for 10 steps ahead
predicted_S = mean(S_particles);
predicted_E = mean(E_particles);
predicted_I = mean(I_particles);

% Predict the evolution for 10 steps ahead
for t = 1:10
    w1 = exp(randn * 0.5);
    w2 = exp(randn * 0.3);
    w3 = exp(randn * 0.3);

    predicted_S = predicted_S + dt * ((-r1 * predicted_E * predicted_S - r2 * predicted_I * predicted_S) * w1 + b1 * (1 - predicted_E - predicted_I - predicted_S) * w2);
    predicted_E = predicted_E + dt * ((r1 * predicted_S * predicted_E + r2 * predicted_I * predicted_S) * w1 - w3 * (b2 + b3) * predicted_E);
    predicted_I = predicted_I + dt * w3 * (b2 * predicted_E - b4 * predicted_I);
end

% Display predicted values
disp('Predicted Evolution 10 Steps Ahead:');
disp(['S: ', num2str(predicted_S)]);
disp(['E: ', num2str(predicted_E)]);
disp(['I: ', num2str(predicted_I)]);

% Plotting the results
figure;
subplot(3, 1, 1);
plot(1:num_steps, avg_S, 'b', 'LineWidth', 1.5);
title('Susceptible Population (S)');
xlabel('Time Step');
ylabel('Fraction of Population');
grid on;

subplot(3, 1, 2);
plot(1:num_steps, avg_E, 'g', 'LineWidth', 1.5);
title('Exposed Population (E)');
xlabel('Time Step');
ylabel('Fraction of Population');
grid on;

subplot(3, 1, 3);
plot(1:num_steps, avg_I, 'r', 'LineWidth', 1.5);
title('Infected Population (I)');
xlabel('Time Step');
ylabel('Fraction of Population');
grid on;

% Plot the predicted evolution after 10 steps
figure;
plot(1:10, predicted_S * ones(10, 1), 'b', 'LineWidth', 1.5);
hold on;
plot(1:10, predicted_E * ones(10, 1), 'g--', 'LineWidth', 1.5);
plot(1:10, predicted_I * ones(10, 1), 'r', 'LineWidth', 1.5);
title('Predicted Evolution for 10 Steps Ahead');
xlabel('Time Step');
ylabel('Fraction of Population');
legend({'Susceptible', 'Exposed', 'Infected'});
grid on;
