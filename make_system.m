function sys = make_system()
%MAKE_SYSTEM Create a data structure to store complete multibody system

sys = struct();

sys.bodies = struct([]);

sys.joints = struct('revolute', struct([]), ...
    'translational', struct([]), ...
    'simple', struct([]), ...
    'simple_driving', struct([]));

sys.solver = struct('t_final', 1, 't_step', 0.01);

end

