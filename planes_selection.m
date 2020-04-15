function [renovation_planes_edge_cell1,renovation_manipulatorbase_planes1,renovation_planes_norm_vector1,renovation_effective_waypoints,renovation_effective_waypaths]=planes_selection(renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_planes_norm_vector,room_effective_waypoints,room_effective_waypaths,panning_distance)

theta_x=-pi/2;
theta_z=pi/2;
m=1;
for i=1:1:size(renovation_planes_edge_cell,2)
    
    nx=renovation_planes_norm_vector{i}(1,1);
    ny=renovation_planes_norm_vector{i}(1,2);
    nz=renovation_planes_norm_vector{i}(1,3);
    sin_theta_y=nx;
    cos_theta_y=ny;
    if sin_theta_y>=0
        if cos_theta_y>=0
            theta_y(i)=asin(sin_theta_y);
        else
            theta_y(i)=pi-asin(sin_theta_y);
        end
    else
        if cos_theta_y>=0
            theta_y(i)=2*pi-asin(abs(sin_theta_y));
        else
            theta_y(i)=pi+asin(abs(sin_theta_y));
        end
    end
    
    if i==6
        renovation_planes_edge_cell1{m}=renovation_planes_edge_cell{i};
        renovation_manipulatorbase_planes1{m}=renovation_manipulatorbase_planes{i};
        renovation_planes_norm_vector1{m}(1,1:3)=renovation_planes_norm_vector{i}(1,1:3);

        a=renovation_planes_norm_vector{i}(1,1);
        b=renovation_planes_norm_vector{i}(1,2);
        c=renovation_planes_norm_vector{i}(1,3);
        
        x0=renovation_planes_edge_cell{i}(1,1);
        y0=renovation_planes_edge_cell{i}(1,2);
        z0=renovation_planes_edge_cell{i}(1,3);
        
        x1=room_effective_waypoints{i}(1,1);
        y1=room_effective_waypoints{i}(1,2);
        z1=room_effective_waypoints{i}(1,3);
        
        d1=-(a*x0+b*y0+c*z0);
        d2=-(a*x1+b*y1+c*z1);
        
        for j=1:1:size(room_effective_waypoints{i},1)
            renovation_effective_waypoints{m}(j,1)=room_effective_waypoints{i}(j,1)-sign(d1-d2)*a/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_effective_waypoints{m}(j,2)=room_effective_waypoints{i}(j,2)-sign(d1-d2)*b/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_effective_waypoints{m}(j,3)=room_effective_waypoints{i}(j,3)-sign(d1-d2)*c/sqrt(a^2+b^2+c^2)*panning_distance;
            renovation_effective_waypoints{m}(j,4:6)=[theta_x,theta_y(i),theta_z];
        end
        for j=1:1:size(room_effective_waypaths{i},1)
            for k=1:1:2
                renovation_effective_waypaths{m}(j,3*k-2)=room_effective_waypaths{i}(j,3*k-2)-sign(d1-d2)*a/sqrt(a^2+b^2+c^2)*panning_distance;
                renovation_effective_waypaths{m}(j,3*k-1)=room_effective_waypaths{i}(j,3*k-1)-sign(d1-d2)*b/sqrt(a^2+b^2+c^2)*panning_distance;
                renovation_effective_waypaths{m}(j,3*k)=room_effective_waypaths{i}(j,3*k)-sign(d1-d2)*c/sqrt(a^2+b^2+c^2)*panning_distance;
            end
        end
        m=m+1;
    end
end
%% add orientations into renovation path waypoints 


end
