clear; clc; close all;

% Intersection parameters
L = 0;
sNS = 1800;
qNS = 600;
sEW = 1800;
qEW = 500;
Cmin = 40;
Cmax = 120;
gmin = 10;

% Symbolic math for Webster formula
syms g c q s real
x = (q * c) / (s * g);
webster = (c * (1 - g/c)^2) / (2 * (1 - q/s)) + (x^2) / (2 * q * (1 - x));
dg = diff(webster, g);
dc = diff(webster, c);
D = matlabFunction(webster, 'Vars', {g, c, q, s});
mg = matlabFunction(dg, 'Vars', {g, c, q, s});
mc = matlabFunction(dc, 'Vars', {g, c, q, s});
disp('Symbolic math setup complete.');

% Generating Linear Constraints (A*x <= b)
A = []; b = [];
Cs = linspace(Cmin, Cmax, 20);
Sp = 0.1:0.01:1.8;
i = 0;
for Cval = Cs
    for s = Sp
        % NS Direction
        gval = Cval * s;
        xNS = (qNS * Cval) / (sNS * gval);
        if xNS < 0.95 && gval >= gmin
            D0 = D(gval, Cval, qNS, sNS);
            m_g = mg(gval, Cval, qNS, sNS);
            m_c = mc(gval, Cval, qNS, sNS);
            row = [m_g, 0, m_c, -1, 0];
            rhs = -(D0 - m_g*gval - m_c*Cval);
            A = [A; row];
            b = [b; rhs];
            i = i + 1;
        end
        
        % EW Direction
        gval_ew = Cval * (1-s) - L; 
        xEW = (qEW * Cval) / (sEW * gval_ew);
        if xEW < 0.95 && gval_ew >= gmin
            D0 = D(gval_ew, Cval, qEW, sEW);
            m_g = mg(gval_ew, Cval, qEW, sEW);
            m_c = mc(gval_ew, Cval, qEW, sEW);
            row = [0, m_g, m_c, 0, -1];
            rhs = -(D0 - m_g*gval_ew - m_c*Cval);
            A = [A; row];
            b = [b; rhs];
            i = i + 1;
        end
    end
end
disp(['Generated ', num2str(i), ' constraints.']);

% Simplex (linprog) Setup
% X = [gNS; gEW; C; zNS; zEW]
f = [0; 0; 0; qNS; qEW];
Aeq = [1, 1, -1, 0, 0];
beq = -L;
lb = [gmin; gmin; Cmin; 0; 0];
ub = [Cmax; Cmax; Cmax; Inf; Inf];
opt = optimoptions('linprog','Display','off');

% Run Optimization
[x, val, flag] = linprog(f, A, b, Aeq, beq, lb, ub, opt);

% Display Results
gNS = x(1);
gEW = x(2);
C_opt = x(3);
zNS = x(4);
zEW = x(5);
fprintf('\n--- Simplex Results ---\n');
fprintf('Optimal C: %.2f s\n', C_opt);
fprintf('Optimal gNS: %.4f s\n', gNS);
fprintf('Optimal gEW: %.4f s\n', gEW);
fprintf('Total Delay: %.2f h/h\n', val/3600);

% Error Check vs Real Curve
realZNS = D(gNS, C_opt, qNS, sNS);
fprintf('NS Delay Error: %.4f sec\n', realZNS - zNS);

% Comparison with real minimum
fun = @(x_nl) qNS * D(x_nl(1), x_nl(3), qNS, sNS) + qEW * D(x_nl(2), x_nl(3), qEW, sEW);
Aeq_nl = [1, 1, -1]; beq_nl = -L;
lb_nl = [gmin, gmin, Cmin];
ub_nl = [Inf, Inf, Cmax];
x0 = [40, 40, 90];
opt_nl = optimoptions('fmincon', 'Display', 'off');
x_exact = fmincon(fun, x0, [], [], Aeq_nl, beq_nl, lb_nl, ub_nl, [], opt_nl);

fprintf('\n--- Exact vs Simplex Comparison ---\n');
fprintf('Solver    | C (s) | gNS (s) | gEW (s)\n');
fprintf('----------|-------|---------|--------\n');
fprintf('Simplex   | %5.2f | %7.4f | %7.4f\n', C_opt, gNS, gEW);
fprintf('fmincon   | %5.2f | %7.4f | %7.4f\n', x_exact(3), x_exact(1), x_exact(2));