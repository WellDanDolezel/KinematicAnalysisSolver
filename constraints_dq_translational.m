function Cq = constraints_dq_translational(sys, q)
%CONSTRAINTS_REVOLUTE Compute constraints for the translational joints
Cq = zeros(2 * length(sys.joints.translational), length(q));
c_id = 0;


for j = sys.joints.translational
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    ri = qi(1:2);
    rj = qj(1:2);
    Ai = rot(qi(3));
    Aj = rot(qj(3));
    Pi = ri + Ai*j.s_i2;
    Qi = ri + Ai*j.s_i1;
    Pj = rj + Aj*j.s_j;

    % based on table 4.2 in Nikravesh
    mat_1 = [Pi(2) - Qi(2), -(Pi(1) - Qi(1)), ...
        -(Pj(1)-ri(1))*(Pi(1)-Qi(1))-(Pj(2)-ri(2))*(Pi(2)-Qi(2)); 0, 0, 1];

    mat_2 = [-(Pi(2) - Qi(2)), Pi(1) - Qi(1), ...
        (Pj(1)-rj(1))*(Pi(1)-Qi(1))+(Pj(2)-rj(2))*(Pi(2)-Qi(2)); 0, 0, -1];

    Cq(c_id + (1:2), j.body_i_qidx) = mat_1;
    Cq(c_id + (1:2), j.body_j_qidx) = mat_2;
    c_id = c_id + 2;
end

end

