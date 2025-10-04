%%
% Ερώτημα 5
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
syms U1 U2 U3 U4
syms phi theta psi 

x1_dot = x2;
x2_dot = p1*x4*x6 + p3*U2;
x3_dot = x4;
x4_dot = p4*x2*x6 + p6*U3;
x5_dot = x6;
x6_dot = p7*x2*x4 + p8*U4;
x7_dot = x8;
x8_dot = (cos(x1)*sin(x3)*cos(x5) + sin(x1)*sin(x5))*(1/m)*U1;
x9_dot = x10;
x10_dot = (-cos(x1)*sin(x3)*sin(x5) + sin(x1)*cos(x5))*(1/m)*U1;
x11_dot = x12;
x12_dot = -g + (cos(x3)*cos(x1))*(1/m)*U1;

% Ορισμός Διανύσματος Κατάστασης X
X = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12];
% Ορισμός Διανύσματος Εισόδων 
U = [U1; U2; U3; U4];
% Εξισώσεις Κατάστασης του Drone
f = [x1_dot; x2_dot; x3_dot; x4_dot; x5_dot; x6_dot; x7_dot; x8_dot; x9_dot; x10_dot; x11_dot; x12_dot];

% Υπολογισμός του γραμμικοποιημένου μοντέλου
A = jacobian(f, X);  
B = jacobian(f, U);  

% Ορισμός Σημείου Ισορροπίας 
x_eq = zeros(12, 1); % Επιλέξαμε x_eq = 0
u_eq = [m*g; 0; 0; 0]; % Απαιτείται δύναμη U1=mg για να έχουμε hover

% Αντικαθιστώ στα σημεία ισορροπίας και μετατρέπω σε double
A_lin = double(subs(A, [X; U], [x_eq; u_eq]));
disp('Linearized matrix A:');
disp(A_lin);
B_lin = double(subs(B, [X; U], [x_eq; u_eq]));
disp('Linearized matrix B:');
disp(B_lin);
%%
% Ερώτημα 6
% Ορισμός εξόδων (x7, x9, x11, x5) όπως ορίζει η εκφώνηση
C = [0 0 0 0 0 0 1 0 0 0 0 0;  % x7
     0 0 0 0 0 0 0 0 1 0 0 0;  % x9
     0 0 0 0 0 0 0 0 0 0 1 0;  % x11
     0 0 0 0 1 0 0 0 0 0 0 0]; % x5
D = zeros(4, 4);

% Ορισμός Συστήματος Κατάστασης 
sys_state = ss(A_lin, B_lin, C, D);

% Συνάρτηση Μεταφοράς
tf_system = tf(sys_state);
disp('Transfer Function:');
tf_system

%%
% Ερώτημα 7: Σχεδιασμός Ελεγκτή με LQ
% Step 1: Calculate the controllability matrix and rank
Co = ctrb(A_lin, B_lin);
rank_Co = rank(Co)  % Should be less than full rank (12) if uncontrollable states exist


%Q = diag([1 1 1 1 1 1 1 1 1 1 1 1]);
%R = diag([1 1 1 1]);
Q = diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]);
R = diag([100, 100, 100, 100]);
%Q = diag([10,1,10,1,10,1,50,1,50,1,50,1]);
%R = diag([0.1, 0.1, 0.1, 0.1]);

% Επίλυση της αλγεβρικής εξίσωσης Riccati για τον πίνακα P
[X, K_LQ, ~] = icare(A_lin, B_lin, Q, R);

% Δημιουργία του Συστήματος Κλειστού Βρόχου
A_cl = A_lin - (B_lin * K_LQ);
eig_values = eig(A_cl)

C_cl = eye(size(A_lin));
D_cl = zeros(size(B_lin));

sys_cl = ss(A_cl, B_lin, C_cl, D_cl);

% Προσομοίωση του Συστήματος Κλειστού Βρόχου
t = 0:0.005:10;  
x0 = [0.1; 0; 0.1; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % Αρχικές συνθήκες

% Προσομοίωση του κλειστού βρόχου συστήματος με αρχικές συνθήκες
[y, t, x] = initial(sys_cl, x0, t);

figure;
plot(t, x);
title('Closed-Loop State Response of the System with LQ Control');
xlabel('Time');
ylabel('State Values');
legend('x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'x10', 'x11', 'x12');
grid on;

%%
% Question 8
% Parameters for simulation
sim_duration = 9; % Total simulation duration in seconds
dt = 0.01; % Time step for each iteration

% Define time vector for each iteration and total simulation time
t_iter = 0:dt:sim_duration;
num_steps = length(t_iter);

positions = zeros(num_steps * size(waypoints, 1), 3); % Store positions
index = 1;

% Reference states for each iteration
waypoints = [
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 1 0 1 0;
    0 0 0 0 0 0 1 0 1 0 1 0;
    0 0 0 0 0 0 1 0 1 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
];

% Moderate values for Q_LQR and R_LQR to improve numerical stability
Q_LQR = diag([100, 1, 1, 1000, 1, 10, 1000, 100, 1, 1, 1, 1]);
R_LQR = diag([1, 10, 1000, 1]);

% Solve the Riccati equation for LQR gain matrix
[X, K_LQR, ~] = icare(A_lin, B_lin, Q_LQR, R_LQR);

% Initial state
x0 = [phi_0; phi_dot_0; th_0; th_dot_0; psi_0; psi_dot_0; x_0; x_dot_0; y_0; y_dot_0; z_0; z_dot_0];

% Prepare arrays for storing simulation results
x_all = [];
t_all = [];

% Loop through each reference trajectory in waypoints
for j = 1:size(waypoints, 1)
    x_ref = waypoints(j, :)';
    
    % Compute steady-state input for reference state
    u_ref = -pinv(B_lin) * (A_lin * x_ref);

    % Initialize state matrix for the current iteration
    x = zeros(num_steps, length(x0));
    x(1, :) = x0';

    % Time vector for the current iteration
    t_current = t_iter + (j - 1) * sim_duration;

    % Simulate system with discrete time steps
    for i = 1:num_steps - 1
        % Calculate control input with reference tracking
        u = -K_LQR * (x(i, :)' - x_ref) + u_ref;
        
        % Update state using Euler integration
        x_dot = A_lin * x(i, :)' + B_lin * u;
        x(i + 1, :) = x(i, :) + x_dot' * dt;
        
        % Store the position for plotting
        positions(index, :) = x(i, [7, 9, 11])';
        index = index + 1;
    end

    % Append results of the current iteration
    x_all = [x_all; x];
    t_all = [t_all, t_current];
    
    % Use final state of the current iteration as the initial state for the next
    x0 = x(end, :)';
end

% Plot results
figure;
plot(t_all, x_all(:, 7), 'r', 'LineWidth', 1.5); % x-state
hold on;
plot(t_all, x_all(:, 9), 'g', 'LineWidth', 1.5); % y-state
plot(t_all, x_all(:, 11), 'b', 'LineWidth', 1.5); % z-state
xlabel('Time (s)');
ylabel('State Variables');
title('State Response for x, y, and z');
legend('x', 'y', 'z');
grid on;
hold off

% Remove unused positions
positions = positions(1:index-1, :);

% Plotting the path
figure;
plot3(positions(:,1), positions(:,2), positions(:,3), 'b', 'LineWidth', 1.5);
hold on;
plot3(waypoints(:,7), waypoints(:,9), waypoints(:,11), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Drone Path Following using LQR Control');
legend('Drone Path', 'Waypoints');
hold off

