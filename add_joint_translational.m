function sys = add_joint_translational(sys, body_i, ...
    body_j, s_i1, s_i2, s_j, init_rot)
%ADD_JOINT_TRANSLATIONAL Add translational joint definition to the system
%INPUT:     sys ... system
%           body_i ... 1st body for the joint
%           body_j ... 2nd body for the joint
%           s_i1 ...   1st point on the line of translation on body i (lcs)
%           s_i1 ...   2st point on the line of translation on body i (lcs)
%           s_j ...   point on the line of translation on body j (lcs)
%           init_rot ... vector of initial rotations of i and j
    arguments
        sys (1,1) struct
        body_i (1,1) string
        body_j (1,1) string
        s_i1 (2,1) double = [0; 0]
        s_i2 (2,1) double = [0; 0]
        s_j (2,1) double = [0; 0] 
        init_rot (2,1) double = [0; 0] 
        
    end
    % Manual checking of bodies names
    check_body_exists(sys, body_i)
    check_body_exists(sys, body_j)
    
    joint = struct();
    joint.body_i_qidx = body_name_to_qidx(sys, body_i);
    joint.body_j_qidx = body_name_to_qidx(sys, body_j);
    joint.s_i1 = s_i1;
    joint.s_i2 = s_i2;
    joint.s_j = s_j;
    joint.init_rot = init_rot;

    sys.joints.translational = [sys.joints.translational, joint];
end
