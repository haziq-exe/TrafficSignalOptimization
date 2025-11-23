clear; clc; close all;

% num of tangent planes = (density x density)
density = 4;


s = 1800;
q = 600;
Cmin = 40;
Cmax = 120;
gmin = 10;


syms g c real
x = (q * c) / (s * g);
webster = (c * (1 - g/c)^2) / (2 * (1 - q/s)) + (x^2) / (2 * q * (1 - x));
dg = diff(webster, g);
dc = diff(webster, c);
D = matlabFunction(webster, 'Vars', {g, c});
mg = matlabFunction(dg, 'Vars', {g, c});
mc = matlabFunction(dc, 'Vars', {g, c});

% Real delay surface
figure('Name', 'Webster Linearization', 'Color', 'w');
[Cm, gm] = meshgrid(linspace(Cmin, Cmax, 60), linspace(gmin, 60, 60));
Z = zeros(size(Cm));

for i = 1:numel(Cm)
    if (q * Cm(i)) / (s * gm(i)) < 0.98
        Z(i) = D(gm(i), Cm(i));
    else
        Z(i) = NaN;
    end
end

surf(Cm, gm, Z, 'FaceColor', 'c', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold on;

% Sample points
sampleC = linspace(50, 110, density);
sampleG = linspace(20, 50, density);

% Plot the tangent planes
for C0 = sampleC
    for g0 = sampleG
        
        if (q * C0) / (s * g0) < 0.95
            
            D0 = D(g0, C0);
            m_g = mg(g0, C0);
            m_c = mc(g0, C0);
            
            sC = 15;
            sG = 10;
            [Cp, gp] = meshgrid([C0-sC, C0+sC], [g0-sG, g0+sG]);
            
            % Plane equation: Z = D0 + m_g*(g - g0) + m_c*(C - C0)
            Zp = D0 + m_g.*(gp - g0) + m_c.*(Cp - C0);
            
            surf(Cp, gp, Zp, 'FaceColor', 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r', 'MeshStyle', 'row');
            plot3(C0, g0, D0, 'k.', 'MarkerSize', 15);
        end
    end
end


title('Linear Approximation of Non-Linear Delay');
xlabel('Cycle Length (C)');
ylabel('Green Time (g)');
zlabel('Delay (Z)');
axis([Cmin Cmax gmin 60 0 50]);
grid on;
view(120, 30);
rotate3d on;