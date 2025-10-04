
% Calculate MSE 
MSE_Bayesian = mean((x_true - x_Bayesian).^2);
MSE_EKF = mean((x_true - x_est).^2);
MSE_PF = mean((x_true - x_PF).^2);

fprintf('MSE (Bayesian Filter): %.4f\n', MSE_Bayesian);
fprintf('MSE (EKF): %.4f\n', MSE_EKF);
fprintf('MSE (Particle Filter): %.4f\n', MSE_PF);

% Γραφική απεικόνιση
figure;
plot(1:N_steps, x_true, 'k--', 'LineWidth', 2); hold on;
plot(1:N_steps, x_Bayesian, 'r', 'LineWidth', 1.5);
plot(1:N_steps, x_est, 'b', 'LineWidth', 1.5);
plot(1:N_steps, x_PF, 'g', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('State');
title('Comparison of Estimates: Bayesian Filter, EKF, PF');
legend('True State', 'Bayesian Filter', 'EKF', 'Particle Filter');
grid on;