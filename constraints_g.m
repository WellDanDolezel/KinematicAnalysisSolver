function g = constraints_g(sys, q, qd, t)
%CONSTRAINTS_G Compute g vector for calculation of acceleration

g = zeros(length(q), 1);
g_idx = 0;

for j = sys.joints.revolute
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    qdi = qd(j.body_i_qidx);
    qdj = qd(j.body_j_qidx);
    Ai = rot(qi(3));
    Aj = rot(qj(3));    fiid = qdi(3);
    fijd = qdj(3);
    g(g_idx + (1:2)) = Ai * j.s_i .* fiid ^ 2 - Aj * j.s_j .* fijd ^ 2;
    g_idx = g_idx + 2;
end

for j = sys.joints.translational
    qi = q(j.body_i_qidx);
    qj = q(j.body_j_qidx);
    ri = qi(1:2);
    qdi = qd(j.body_i_qidx);
    qdj = qd(j.body_j_qidx);
    Ai = rot(qi(3));


    Pi = ri + Ai*j.s_i2;
    Qi = ri + Ai*j.s_i1;

    % based on table 4.3 in Nikravesh
    bracket_1 = ((Pi(1)-Qi(1))*(qdi(1)-qdj(1))+(Pi(2)-Qi(2))*(qdi(2)-qdj(2)))*qdi(3);
    bracket_2 = ((Pi(1)-Qi(1))*(qi(2)-qj(2))-(Pi(2)-Qi(2))*(qi(1)-qj(1)))*qdi(3)^2;
    g(g_idx + 1) = -2*bracket_1-bracket_2;
    g_idx = g_idx + 2;
end

g_idx = g_idx + length(sys.joints.simple);

for j = sys.joints.simple_driving
    g(g_idx + 1) = -j.coord_fun_dtt(t);
    g_idx = g_idx + 1;
end



end

