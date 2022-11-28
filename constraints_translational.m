function C = constraints_translational(sys, q)
%CONSTRAINTS_TRANSLATIONAL Compute constraints for the translational joints
C = zeros(2 * length(sys.joints.translational), 1);
c_id = 0;

for j = sys.joints.translational
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    ri = qi(1:2);
    rj = qj(1:2);
    phi_i = qi(3);
    phi_j = qj(3);
    Ai = rot(qi(3));
    Aj = rot(qj(3));

    Pi = ri + Ai*j.s_i2;
    Qi = ri + Ai*j.s_i1;
    Pj = rj + Aj*j.s_j;

    % line of translation
    d = Pj - Pi;
    
    % perpendicular vector to line of translation based on two points on body i
    n = [-(Pi(2)-Qi(2)); Pi(1)-Qi(1)];

    C(c_id + 1) = n'*d;
    C(c_id + 2) = phi_i-phi_j-(j.init_rot(1)-j.init_rot(2));
    c_id = c_id + 2;
end

end

