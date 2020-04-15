%% rpy test
% mat1=rpy2r(pi/2,0,pi/2,'arm')
% rpy=tr2rpy(mat1,'xyz')
% mat2=rpy2r(rpy(1),rpy(2),rpy(3),'arm')
% 
% theta=pi/4;
% mat3=rotz(pi+theta)'*rotx(pi/2)*roty(theta)*rotz(pi/2)
% rpy=tr2rpy(mat3,'xyz')
% mat4=rpy2r(rpy(1),rpy(2),rpy(3),'arm')

% mat1=rotx(pi/2);
% mat3=rotz(pi/2);
% costheta_y=-1;
% sintheta_y=0;
% mat2=[costheta_y 0 sintheta_y;0 1 0;-sintheta_y 0 costheta_y];
% rpy_endeffector_to_world=mat1*mat2*mat3;
% costheta_z=-1;
% sintheta_z=0;
% rpy_base_to_world=[costheta_z -sintheta_z 0;sintheta_z costheta_z 0;0 0 1];
% rpy_endeffector_to_base=rpy_base_to_world'*rpy_endeffector_to_world;

cos_theta_z=0;
sin_theta_z=-1;
if sin_theta_z>=0
    if cos_theta_z>=0
        base_theta_z=asin(sin_theta_z);
    else
        base_theta_z=pi-asin(sin_theta_z);
    end
else
    if cos_theta_z>=0
        base_theta_z=2*pi-asin(abs(sin_theta_z));
    else
        base_theta_z=pi+asin(abs(sin_theta_z));
    end
end

base_theta_z





