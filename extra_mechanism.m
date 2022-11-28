clear, clc, close all

%% PREPROCESSOR
% Globally we have a complete multibody system
% It must contain bodies, joints, analysis settings
sys = make_system();

% Bodies
% location, orientation, name 

sys = add_body(sys, "ground");
sys = add_body(sys, "crank", [1.5*cos(deg2rad(30)), 1.5*sin(deg2rad(30))], deg2rad(30));
sys = add_body(sys, "slider", [3*cos(deg2rad(30)), 3*sin(deg2rad(30))], deg2rad(60));
sys = add_body(sys, "arm", [6*cos(deg2rad(30)),0], deg2rad(60));

% Joints - kinematic (revolute, translational and simple in this case)
sys = add_joint_revolute(sys, "ground", "crank", [0; 0], [-1.5; 0]);
sys = add_joint_revolute(sys, "crank", "slider", [1.5; 0], [0; 0]);
sys = add_joint_revolute(sys, "arm", "ground", [0; 0], [6*cos(deg2rad(30)); 0]);

sys = add_joint_translational(sys,"slider","arm", [0; 0], [0; 1],...
    [0, 1], [deg2rad(60), deg2rad(60)]);

sys = add_joint_simple(sys, "ground", "x");
sys = add_joint_simple(sys, "ground", "y");
sys = add_joint_simple(sys, "ground", "fi");

sys = add_joint_simple_driving(sys, "crank", "fi", ...,
    @(t) deg2rad(30) - 1.2 * t, ...
    @(t) - 1.2,...
    @(t) 0);

sys = set_solver_settings(sys, 10, 0.001);

%% SOLVER fsolve

% [Tf, Qf] = solve_kinematics_fsolve(sys);

%% SOLVER NR

[T, Q, Qd, Qdd] = solve_kinematics_NR(sys);

%% POSTPROCESSING
pidx = 9;
figure(1), set(gcf,'color','w','Position',[200 200 600 175]);
plot(T, Q(pidx, :), T, Qd(pidx, :), T, Qdd(pidx, :),'LineWidth',2)
legend('Pos','Vel','Acc')
xlabel('t [s]')
grid on
% axis equal

