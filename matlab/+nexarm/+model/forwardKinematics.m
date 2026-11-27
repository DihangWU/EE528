function [tip,points,pose] = forwardKinematics(model,q)
%FORWARDKINEMATICS Gripper pose and pen tip in base_link coordinates, metres.
% J6 opens the gripper; it does not transform the gripper-mounted pen.
validateattributes(q,{'numeric'},{'real','finite','vector','numel',6});
q=q(:).'; pose=eye(4); points=zeros(8,3);
for j=1:6
    pose=pose*model.origins(:,:,j);
    if model.revolute(j)
        vector=model.axes(j,:)*q(j); angle=norm(vector);
        if angle>0
            axis=vector/angle;
            k=[0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
            rotation=eye(4); rotation(1:3,1:3)=eye(3)+sin(angle)*k+(1-cos(angle))*(k*k);
            pose=pose*rotation;
        end
    end
    points(j+1,:)=pose(1:3,4).';
end
mount=pose(1:3,4)+model.pen_forward*pose(1:3,1);
tip=(mount-model.pen_length*pose(1:3,3)).'; points(8,:)=mount.';
end
