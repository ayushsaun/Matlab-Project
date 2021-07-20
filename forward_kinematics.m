%% for calculating forward kinematics

function x = forward_kinematics(theta,puma)
%     n = 6;
%     lengths = [0.1 0.1 0.1 0.1 0.1 0.1];
%     pts = r_planar(n, lengths, theta);
    pts = zeros(4,4);
    pts(:,:) = puma.fkine(theta);
    x(1) = pts(1,4);
    x(2) = pts(2,4);
    x(3) = pts(3,4);
end
