w_std = sqrt(10); % Covariance of w_k
v_std = sqrt(1);  % Covariance of v_k

x = randn(1);
y = [];

N_steps = 100;
alpha = 0;
beta = 25;

for k = 1:N_steps
    w = randn(1) * w_std;
    x(k+1) = 0.5 * x(k) + beta * x(k) / (1 + x(k) ^ 2) + 8 * cos(1.2 * k) + w;
    
    v = randn(1) * v_std;
    y(k) = alpha * x(k) + x(k) ^ 2 / 20 + v;
end

x=x(1:end-1);