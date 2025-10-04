% Define possible states, x, between X_min and X_max
X_min = -35;
X_max = 35;
N = 1001;

x_values = linspace(X_min, X_max, N);
P_Prior = zeros(N, 1);
x_0_mean = 0; % Mean value 
x_0_std = 1; % Standard deviation
mean_values = zeros(N, 1); % To track means over steps

% Initialize prior (Gaussian)
for i=1:N
    P_Prior(i) = (1 / (sqrt(2 * pi * x_0_std ^2))) * exp(-(x_values(i) - x_0_mean) ^ 2 / (2 * x_0_std ^ 2));
end

x_Bayesian = zeros(1, N_steps); % Store the expected values (6th question)

% recursive Bayesian Filtering
for k=1:N_steps

    y_meas = y(k);
    
    % Correction Step
    %likelihood_unnormalized = zeros(N, 1);
    p_posterior_unnormalized = zeros(N, 1);
    for x_val_ind = 1:N
        x_val_i = x_values(x_val_ind); 

        % Likelihood Computation
        y_pred = alpha * x_val_i + x_val_i ^ 2 / 20;
        likelihood_unnormalized = (1 / sqrt(2 * pi * v_std ^ 2)) * exp(-(y_meas - y_pred) ^ 2 / (2 * v_std ^ 2));
        
        % Bayes' rule P(x_k|y_k) = P(y_k|x_k)P(x_k)
        p_posterior_unnormalized(x_val_ind) = P_Prior(x_val_ind) * likelihood_unnormalized;
    end
   
    normalize_factor = sum(p_posterior_unnormalized) * (x_values(2) - x_values(1));
    p_posterior = p_posterior_unnormalized / normalize_factor; % Normalization

    
    x_Bayesian(k) = sum(x_values .* p_posterior' * (x_values(2) - x_values(1)));
    
    % Prediction Step
    p_x_new = zeros(N, 1);
    for x_val_new_ind = 1:N
        x_val_i_new = x_values(x_val_new_ind);
        
        % Integrating over previous states
        p_x_new_to_integrate = zeros(N, 1);
        for x_val_cur_ind = 1:N
            x_val_i_cur = x_values(x_val_cur_ind);
            
            % Transition Probability
            x_pred = (0.5 * x_val_i_cur + beta * x_val_i_cur / (1 + x_val_i_cur ^ 2) + (8 * cos(1.2 * k)));
            p_Transition_x_to_x_pl = (1 / (sqrt(2 * pi * w_std ^ 2))) * exp(-(x_val_i_new - x_pred) ^ 2 / (2 * w_std ^ 2));
            
            % Weight by posterior
            p_x_new_to_integrate(x_val_cur_ind) = p_Transition_x_to_x_pl * p_posterior(x_val_cur_ind);
        end
        
        % Sum for new prior
        p_x_new(x_val_new_ind)  = sum(p_x_new_to_integrate) * (x_values(2) - x_values(1));
            
    end
    % Update the prior value -> to new for the next iteration
    P_Prior = p_x_new;
    
    % Visualization
    figure;
    plot(x_values, P_Prior, 'b', 'LineWidth', 1.5);
    hold on;
    plot(x_values, p_posterior, 'r', 'LineWidth', 1.5);
    title(['Time Step ', num2str(k)]);
    legend('Prior', 'Posterior');
    quiver(y_meas, 0, 0, 0.5);
    quiver(x(k), 0, 0, 0.55);
    hold off;
end 
