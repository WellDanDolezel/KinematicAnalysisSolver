function C = constraints(sys, q, t)
% C Assamble vector of constrains
C = [
    constraints_revolute(sys, q)
    constraints_translational(sys, q)
    constraints_simple(sys, q)
    constraints_simple_driving(sys, q, t)
    ];