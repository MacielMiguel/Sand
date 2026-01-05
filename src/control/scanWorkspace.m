clc; close all;

% Geometry
L1 = 0.04; L2 = 0.10; L3 = 0.10;

% Joint limits
th1 = linspace(-0.24,  0.2, 40); % [460, 545] => offset = 0.22
th2 = linspace(-2.55,  -1.06, 40); % [10, 300] => offset = 1.805
th3 = linspace(-0.65,  0.47, 40); % [380, 600] => offset = 0.56

XYZ = zeros(3, numel(th1)*numel(th2)*numel(th3));
YZ = zeros(2, numel(th2)*numel(th3));
idx = 1;
idx_2 = 1;

% Three-dimensional
for t1 = th1
  for t2 = th2
    for t3 = th3
        q = [t1; t2; t3];
        chi = FK_local(q, L1, L2, L3);
        XYZ(:,idx) = chi;
        idx = idx + 1;
    end
  end
end

% Two-dimensional
for t4 = th2
    for t5 = th3
        q = [t4; t5];
        chi = FK_bidi(q, L1, L2, L3);
        YZ(:,idx_2) = chi;
        idx_2 = idx_2 + 1;
    end
end

% ===== Figure 1 : Workspace 3D =====
figure;
scatter3(XYZ(1,:), XYZ(2,:), XYZ(3,:), 3, '.');
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Workspace 3D');

% ===== Figure 2 : Workspace 2D (YZ) =====
figure;
scatter(YZ(1,:), YZ(2,:), 2, '.');
axis equal; grid on;
ylabel('Y'); zlabel('Z');
title('Workspace 2D');

% -----------------------

% Three-dimensional FK
function chi = FK_local(q, L1, L2, L3)
theta1 = q(1); theta2 = q(2); theta3 = q(3);

m = sqrt( (sin(theta3)^2) * (L3^2) + (cos(theta3)*L3 + L2)^2 );
h = sqrt( L1^2 + m^2 );
x = sin( pi/2 - atan(m/L1) - theta1 ) * h;
y = cos( pi/2 - atan(m/L1) - theta1 ) * h;
z = sin( pi - theta2 - atan( (sin(theta3)*L3) / (L2 + cos(theta3)*L3) ) ) * m;

chi = [x; y; z];
end

% Two-dimensional FK
function chi = FK_bidi(q, L1, L2, L3)
theta2 = q(1); theta3 = q(2);

x = 0.042;
m = sqrt(L2^2 + L3^2 + 2*L2*L3*cos(theta3)); 
y = sqrt(L1^2 + m^2 - x^2);

alpha = atan2(L3*sin(theta3), (L2 + L3*cos(theta3)));   
phi   = pi - theta2 - alpha;
z = m * sin(phi);

chi = [y; z];

end

