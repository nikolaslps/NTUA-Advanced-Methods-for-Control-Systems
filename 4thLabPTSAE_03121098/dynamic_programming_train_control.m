function dynamic_programming_train_control
    % Constants
    k1 = 0.5; 
    k2 = 1.0; 
    k3 = 1.0; 
    k4 = 10;
    c1 = 1000; 
    c2 = 1000; 
    x1f = 10;
    R = 0.3;
    I_min = -2; 
    I_max = 2;
    T = 10; % Final time
    N = 100; % Number of time steps

    % Discretization of time, and discretization of grid
    dt = T / N;
    x1_range = linspace(-15, 15, 50); 
    x2_range = linspace(-10, 10, 50);
    [X1, X2] = meshgrid(x1_range, x2_range);
    
    % Cost-to-go matrix
    J = inf(size(X1));
    u_optimal = zeros(size(X1, 1), size(X2, 2), N-1);

    % Final cost function φ(x(t),u(t))
    J_final = c1 * (X1 - x1f).^2 + c2 * X2.^2;
    J(:, :, N) = J_final;
    
    % Convert into NDGRID format for griddedInterpolant function
    X1_nd = X1'; 
    X2_nd = X2'; 
    
    % Backward iteration
    for n = N-1:-1:1
        J_nd = permute(J(:, :, n+1), [2, 1]); % Change dimentions for griddedInterpolant function
        % Handle cases where (x1_next, x2_next) do not exactly match the grid points (x1, x2)
        J_interpolant = griddedInterpolant(X1_nd, X2_nd, J_nd, 'linear', 'nearest');
        
        for i = 1:size(X1, 1)
            for j = 1:size(X2, 2)
                x1 = X1(i, j);
                x2 = X2(i, j);

                % Minimize cost over control input u
                u_range = linspace(I_min, I_max, 20);
                costs = zeros(size(u_range));

                for k = 1:length(u_range)
                    u = u_range(k);
                    x1_next = x1 + x2 * dt;
                    x2_next = x2 + (-k1 * x2 - k2 * x2^2 + k3 * u) * dt;
                    
                                       
                    % Interpolate cost-to-go for the next state (check if
                    % inside grid)
                    if x1_next >= min(x1_range) && x1_next <= max(x1_range) && ...
                       x2_next >= min(x2_range) && x2_next <= max(x2_range)

                        % Estimate the cost-to-go for this next state
                        J_next = J_interpolant(x1_next, x2_next);
                        costs(k) = k4 * x2 * u + R * u^2 + J_next;
                        %costs(k) = c1 * x1^2 + c2 * x2^2 + R * u^2 + J_next;
                    else
                        costs(k) = inf; % Outside bounds
                    end
                end

                % Store minimum cost and optimal control
                [J(i, j, n), idx] = min(costs);
                u_optimal(i, j, n) = u_range(idx);
            end
        end
    end

    % Simulate optimal trajectory
    x1 = 0; 
    x2 = 0;
    x1_traj = zeros(1, N);
    x2_traj = zeros(1, N);
    u_traj = zeros(1, N);

    for n = 1:N-1
        x1_traj(n) = x1;
        x2_traj(n) = x2;

        % Find nearest grid point
        [~, i] = min(abs(x1_range - x1));
        [~, j] = min(abs(x2_range - x2));

        % Apply optimal control
        u = u_optimal(i, j, n);
        u_traj(n) = u;

        % Update state
        x1 = x1 + x2 * dt;
        x2 = x2 + (-k1 * x2 - k2 * x2^2 + k3 * u) * dt;
    end

    % Final state values at t = T
    x1_traj(N) = x1;
    x2_traj(N) = x2;
    
    % Plot results
    t = linspace(0, T, N);

    figure;
    subplot(3, 1, 1);
    plot(t, x1_traj, 'b', 'LineWidth', 1.5);
    title('Position (x_1)'); 
    xlabel('Time (s)'); 
    ylabel('x_1 (m)');

    subplot(3, 1, 2);
    plot(t, x2_traj, 'r', 'LineWidth', 1.5);
    title('Velocity (x_2)'); 
    xlabel('Time (s)'); 
    ylabel('x_2 (m/s)');

    subplot(3, 1, 3);
    plot(t, u_traj, 'g', 'LineWidth', 1.5);
    title('Optimal Control Input (u)'); 
    xlabel('Time (s)'); 
    ylabel('u (A)');
end
