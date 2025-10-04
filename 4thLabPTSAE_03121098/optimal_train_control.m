function optimal_train_control
    % Constants
    k1 = 0.5; k2 = 1.0; k3 = 1.0; k4 = 10;
    c1 = 1000; c2 = 1000; 
    x1f = 10;
    R = 0.3;
    I_min = -2; I_max = 2;
    T = 10; % Final time

    % Initial guess for Boundary Value Problem (BVP)
    solinit = bvpinit(linspace(0, T, 100), @init_guess);

    % Solve the BVP with a 4th order collocation method (bvp4c)
    sol = bvp4c(@ode_system, @boundary_conditions, solinit);

    % Extract solution
    t = sol.x;
    %y = sol.y; 
    x1 = sol.y(1, :); % Position
    x2 = sol.y(2, :); % Velocity
    p1 = sol.y(3, :); 
    p2 = sol.y(4, :);
    
    % Optimal solution 
    len = length(t);
    u_opt = zeros(1, len);
    
    for i = 1:len
        u_opt(i) = opt_controller(p2(i), x2(i));
    end

    % Plot results for State Variables
    figure;
    subplot(2, 1, 1);
    plot(t, x1, 'b', 'LineWidth', 1.5);
    title('Position (x_1)'); 
    xlabel('Time (s)'); 
    ylabel('x_1 (m)');

    subplot(2, 1, 2);
    plot(t, x2, 'r', 'LineWidth', 1.5);
    title('Velocity (x_2)'); 
    xlabel('Time (s)'); 
    ylabel('x_2 (m/s)');
 
    % Plot results for Co-State Variables
    figure;
    subplot(2, 1, 1);
    plot(t, p1, 'b', 'LineWidth', 1.5);
    title('Costate Variable (p_1)'); 
    xlabel('Time (s)'); 
    ylabel('p_1');

    subplot(2, 1, 2);
    plot(t, p2, 'r', 'LineWidth', 1.5);
    title('Costate Variable (p_2)'); 
    xlabel('Time (s)'); 
    ylabel('p_2');
    
    figure;
    plot(t, u_opt, 'b', 'LineWidth', 1.5);
    title('Optimal Control Input (u)'); 
    xlabel('Time (s)'); 
    ylabel('u (A)');

    % Display cost for the output
    disp(' ');
    disp('Calculated cost for the control problem:');
    
    state_difference = (x1(len) - x1f);
    control_signal = u_opt;
    control_penalty = trapz(k4 * x2 .* control_signal + R * (control_signal.^2));

    J = c1 * state_difference^2 + c2 * x2(len)^2 + control_penalty;
    fprintf('The calculated cost is: %.4f\n', J);
    
    
    % ----------------- For question 6 ------------------
    x1_new = zeros(1, len);
    x2_new = zeros(1, len);
    x1_dot_new = zeros(1, len);
    x2_dot_new = zeros(1, len);
    
    dt = T / len;
    x1_new(1) = 0.1;
    x2_new(1) = 0.3;
    
    for i = 1:len
        x1_dot_new(i) = x2_new(i);
        x2_dot_new(i) = - k1 * x2_new(i) - k2 * x2_new(i)^2 + k3 * u_opt(i);
        if i < len
            x1_new(i + 1) = x1_new(i) + x1_dot_new(i) * dt;
            x2_new(i + 1) = x2_new(i) + x2_dot_new(i) * dt;
        end
    end
    
    % Plot results for State Variables
    figure;
    subplot(2, 1, 1);
    plot(t, x1_new, 'b', 'LineWidth', 1.5);
    title('Position (x_1)'); 
    xlabel('Time (s)'); 
    ylabel('x_1 (m)');

    subplot(2, 1, 2);
    plot(t, x2_new, 'r', 'LineWidth', 1.5);
    title('Velocity (x_2)'); 
    xlabel('Time (s)'); 
    ylabel('x_2 (m/s)');
    
    
    % Display new cost for the output
    disp(' ');
    disp('Calculated cost for the new control problem:');
    
    state_difference = (x1_new(len) - x1f);
    control_signal = u_opt;
    control_penalty = trapz(k4 * x2_new .* control_signal + R * (control_signal.^2));

    J_new = c1 * state_difference^2 + c2 * x2_new(len)^2 + control_penalty;
    fprintf('The calculated new cost is: %.4f\n', J_new);
    % -------------------------------------------------
    
    % ----------------- For question 8 ------------------
    estimation = polyfit(t, x2, 5); % Linearization with polynomials of 5th grade
    options = bvpset('RelTol', 1e-6, 'AbsTol', 1e-8); % Increase tolerance
    solinit_Ricatti = bvpinit(linspace(0, T, 100), [1;1;1]);
    sol_Ricatti = bvp4c(@ode_system_Ricatti, @boundary_conditions_Ricatti, solinit_Ricatti, options);
    
    t = sol_Ricatti.x;
    p11 = sol_Ricatti.y(1, :); 
    p22 = sol_Ricatti.y(2, :); 
    p12 = sol_Ricatti.y(3, :); 
    
    % Optimal solution 
    v_opt = zeros(1, len);
    
    y1_new = zeros(1, len);
    y2_new = zeros(1, len);
    y1_dot_new = zeros(1, len);
    y2_dot_new = zeros(1, len);
    
    dt = T / len;
    y1_new(1) = 0.1;
    y2_new(1) = 0.3;
    
    for i = 1:len
        v_opt(i) = v_optimal(y1_new(i),y2_new(i),p12(i),p22(i));
        
        A22 = - k1 - 2 * k2 * x2_new(i);
        
        y1_dot_new(i) = y2_new(i);
        y2_dot_new(i) = A22 * y2_new(i) + k3 * v_opt(i);

        if i < len
            y1_new(i+1) = y1_new(i) + y1_dot_new(i) * dt;
            y2_new(i+1) = y2_new(i) + y2_dot_new(i) * dt;
        end
    end
    
    % Plot results for State Variables
    figure;
    t = linspace(0, T, length(y1_new));
    
    subplot(2, 1, 1);
    plot(t, y1_new, 'b', 'LineWidth', 1.5);
    title('Position (y_1)');
    xlabel('Time (s)');
    ylabel('y_1 (m)');

    subplot(2, 1, 2);
    plot(t, y2_new, 'r', 'LineWidth', 1.5); 
    title('Velocity (y_2)');
    xlabel('Time (s)');
    ylabel('y_2 (m/s)');

    v_opt_modified = u_opt + v_opt;
    
    figure;
    plot(t, v_opt_modified, 'b', 'LineWidth', 1.5);
    hold on;
    plot(t, v_opt, 'r--', 'LineWidth', 1);
    plot(t, u_opt, 'g--', 'LineWidth', 1);
    title('Controllers'); 
    xlabel('Time (s)'); 
    ylabel('(A)');
    legend('Modified Controller', 'Controller', 'Optimal Controller');
    
    % Display new cost for the output
    disp(' ');
    disp('Calculated cost for the linearized control problem:');
    
    state_difference = (x1_new(len) - x1f);
    control_signal = v_opt_modified;
    control_penalty = trapz(k4 * x2_new .* control_signal + R * (control_signal.^2));

    J2 = c1 * state_difference^2 + c2 * x2_new(len)^2 + control_penalty;
    fprintf('The calculated new cost is: %.4f\n', J2);
    % -------------------------------------------------
    
    % NESTED FUNCTIONS
    
    % Nested function for optimal controller
    function u = opt_controller_nobounds(p2, x2)
        u = -(k3 * p2 + k4 * x2)/(2 * R);
    end
    
    % Nested function for optimal controller (set u bounds)
    function u = opt_controller(p2, x2)
        u = -(k3 * p2 + k4 * x2)/(2 * R);
        if u < I_min
            u = I_min;
        elseif u > I_max
             u = I_max;
        end
    end % With that function, when calling ode_system to solve bvp4c we encountered singular Jacobian
    
    % Nested function for ODE system
    function state = ode_system(t, y)
        x1 = y(1); x2 = y(2); p1 = y(3); p2 = y(4);
        
        u = opt_controller_nobounds(p2, x2);
        
        x1_dot = x2;
        x2_dot = - k1 * x2 - k2 * x2^2 + k3 * u;
        p1_dot = 0;
        p2_dot = - k4 * u + k1 * x2 - p1 + 2 * k2 * x2 * p2;

        state = [x1_dot; x2_dot; p1_dot; p2_dot];
    end

    % Nested function for boundary conditions
    function res = boundary_conditions(y0, yfinal)
        res = [
            y0(1);
            y0(2);
            yfinal(3) - 2 * c1 * (yfinal(1) - x1f); 
            yfinal(4) - 2 * c2 * yfinal(2)
        ];
    end

    % Nested function for initial guess
    function guess = init_guess(t)
        guess = [1; 1; 1; 1];
    end

    % --------- Extra Nested Functions for question 8 ----------
    function P = ode_system_Ricatti(t, y)
        p11 = y(1); p22 = y(2); p12 = y(3);
        
        x2 = polyval(estimation, t);
        A22 = - k1 - 2 * k2 * x2;
        
        p11_dot = k3^2 * p12^2 - 2;
        p22_dot = k3^2 * p22^2 - 2 - 2 * (p12 + A22 * p22);
        p12_dot = k3^2 * p12 * p22 - p11 - A22 * p12; 
        
        P = [p11_dot; p22_dot; p12_dot];
    end

    % P(T) = S
    function res = boundary_conditions_Ricatti(y0, yfinal)
        res = [
        yfinal(1) - 20; % Boundary for p11
        yfinal(2) - 20; % Boundary for p22
        yfinal(3)       % Boundary for p12
        ];
    end

    function v = v_optimal(y1, y2, p12, p2)
        v = -k3 * (p12 * y1 + p2 * y2);
    end
    % -------------------------------------------------
end
