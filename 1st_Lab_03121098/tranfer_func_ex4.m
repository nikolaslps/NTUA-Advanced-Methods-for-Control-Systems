V_in=9;
L=.3e-3;
C=.07e-3;
R=10;

D_ss = 0.6;

%For question 4
Vo_ss = V_in/(1-D_ss);
iL_ss = Vo_ss/(R*(1-D_ss));

%Matrix A
A = [0 -(1-D_ss)/L; (1-D_ss)/C -1/(R*C)];
%Matrix B
B = [Vo_ss/L; -iL_ss/C];
%Matrix C for y = x2
C_mat = [0 1];
%Matrix d_bar
d_bar = [1/L; 0];

%Stage base model
ss(A, B, C_mat, 0);
sys = tf(ss(A, B, C_mat, 0))

%Get zeros of the system
z = zero(sys);

%Check if all zeros have negative real parts -> Minimum Phase
if all(real(z) < 0)
    disp('The system is minimum phase.');
else
    disp('The system is NOT minimum phase.');
end

%For question 5
Aag = [A [0; 0]; C_mat 0];
Bag = [B; 0];

eig(Aag)

%The following gives us the gain values we need
%so that the new system eigenvalues become [-1500, -1800, -200]
%we care about dx_ag/dt=(Aag-Bag*K)*x_ag
K = place(Aag, Bag, [-1500, -1800, -200])

eig(Aag-Bag*K)
 
%For question 6
Q = [0.1, 0, 0; 0, 3, 0; 0, 0, 15000];
R_LQ = 1;

%solution of Riccati
[X, K_LQ, ~] = icare(Aag, Bag, Q, R_LQ)

eig(Aag-Bag*K_LQ)
