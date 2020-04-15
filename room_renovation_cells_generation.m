%function [renovation_cells_edges_classified,renovation_cells_paths_classified,renovationcells_pathswaypoints_classified,renovation_cells_manipulatorbase_positions,manipulator_endeffector_positions]=room_renovation_cells_generation(renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_effective_waypaths,renovation_planes_norm_vector,length_interval,width_interval,path_distance)
function [renovation_cells_clustering_waypaths,renovation_cells_waypioints_onwaypath,renovation_cells_waypioints_onpath3,renovation_cells_manipulatorbase_positions3,manipulator_endeffector_positions_onpath3]=room_renovation_cells_generation(room_plane_edge_cell,renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_effective_waypaths,renovation_planes_norm_vector,length_interval,width_interval,path_distance)

%% obtain two types of norm vector of intersection plane norm vector for the renovation planes edges 
for i=1:1:size(renovation_planes_norm_vector,2)
    renovation_planes_point(i,1:3)=renovation_planes_edge_cell{i}(1,1:3);
    renovation_planes_parameters(i,1:3)=renovation_planes_norm_vector{i}(1,1:3);
    renovation_planes_parameters(i,4)=-renovation_planes_norm_vector{i}(1,1:3)*renovation_planes_point(i,1:3)';
end
intersect_plane_norm_vector=zeros(size(renovation_planes_norm_vector,2),3);
intersect_plane_norm_vector1=zeros(size(renovation_planes_norm_vector,2),3);
intersect_plane_norm_vector2=zeros(size(renovation_planes_norm_vector,2),3);
for i=1:1:size(intersect_plane_norm_vector,1)
    sin_theta=-renovation_planes_parameters(i,1);
    cos_theta=sqrt(1-sin_theta^2);
    if cos_theta~=0
        sin_beta=renovation_planes_parameters(i,2)/cos_theta;
        cos_beta=renovation_planes_parameters(i,3)/cos_theta;
    else
        sin_beta=0;
        cos_beta=1;
    end
    intersect_plane_norm_vector1(i,1:3)=[cos_theta, sin_theta*sin_beta, sin_theta*cos_beta];
    intersect_plane_norm_vector2(i,1:3)=[0, cos_beta, -sin_beta];
end

%% obtain dmin and dmax of intersection plane for the renovation planes edges.
for i=1:1:size(renovation_planes_edge_cell,2)
    for j=1:1:size(renovation_planes_edge_cell{i},1)
        renovation_planes_edge_points{i}(2*j-1,1:3)=renovation_planes_edge_cell{i}(j,1:3);
        renovation_planes_edge_points{i}(2*j,1:3)=renovation_planes_edge_cell{i}(j,4:6);
    end
    renovation_planes_edge_points{i}=unique(renovation_planes_edge_points{i},'rows');
end

for i=1:1:size(renovation_planes_edge_points,2)
    a=renovation_planes_norm_vector{i}(1,1);
    b=renovation_planes_norm_vector{i}(1,2);
    a1=intersect_plane_norm_vector1(i,1);
    b1=intersect_plane_norm_vector1(i,2);
    a2=intersect_plane_norm_vector2(i,1);
    b2=intersect_plane_norm_vector2(i,2);
    if a1*a1+b1*b1==1
        intersect_plane_norm_vector(i,1:3)=intersect_plane_norm_vector1(i,1:3);
        critical_plane_norm_vector(i,1:3)=intersect_plane_norm_vector2(i,1:3);
    end
    if a2*a2+b2*b2==1
        intersect_plane_norm_vector(i,1:3)=intersect_plane_norm_vector2(i,1:3);
        critical_plane_norm_vector(i,1:3)=intersect_plane_norm_vector1(i,1:3);
    end
    for j=1:1:size(renovation_planes_edge_points{i},1)  
        intersect_plane_d_candidate{i}(1,j)=-intersect_plane_norm_vector(i,1:3)*renovation_planes_edge_points{i}(j,1:3)';
    end
    intersect_plane_dmin_max{i}(1,1)=min(intersect_plane_d_candidate{i});
    intersect_plane_dmin_max{i}(1,2)=max(intersect_plane_d_candidate{i});
end

for i=1:1:size(intersect_plane_dmin_max,2)
    dmin=intersect_plane_dmin_max{i}(1,1);
    dmax=intersect_plane_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax-dmin)/length_interval);
    for j=1:1:plane_num 
        intersect_plane_d{i}(1,j)=dmin+j*length_interval;
    end
end

for i=1:1:size(renovation_planes_edge_points,2)
    for j=1:1:size(renovation_planes_edge_points{i},1)  
        intersect_plane2_d_candidate{i}(1,j)=-critical_plane_norm_vector(i,1:3)*renovation_planes_edge_points{i}(j,1:3)';
    end
    intersect_plane2_dmin_max{i}(1,1)=min(intersect_plane2_d_candidate{i});
    intersect_plane2_dmin_max{i}(1,2)=max(intersect_plane2_d_candidate{i});
end
for i=1:1:size(intersect_plane2_dmin_max,2)
    dmin_intersect_plane2=intersect_plane2_dmin_max{i}(1,1);
    dmax_intersect_plane2=intersect_plane2_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax_intersect_plane2-dmin_intersect_plane2)/width_interval);
    for j=1:1:plane_num 
        intersect_plane2_d{i}(1,j)=dmin_intersect_plane2+j*width_interval;
    end
end

%% input: renovation waypoints for each plane 
%% output: renovation waypoints for each cell

renovation_effective_waypaths;


%% gather renovation cells edges into renovation cells, the output is:renovation_cells_waypaths
% input: edge_points_segment
% input: intersect_plane_norm_vector,intersect_plane_d
% input: critical_plane_norm_vector,intersect_plane2_d
% output: renovation_cells_waypaths

for i=1:1:size(intersect_plane_d,2)
    cell_num=1;
    if i==1
        h_num=size(intersect_plane_d{i},2)+1;
    else
        h_num=size(intersect_plane_d{i},2);
    end
    for j=1:1:h_num
        for k=1:1:size(intersect_plane2_d{i},2)+1
            renovation_cells_edges_num=1;
            for n=1:1:size(renovation_effective_waypaths{i},1)
                p1=renovation_effective_waypaths{i}(n,1:3);
                p1_d1=-intersect_plane_norm_vector(i,1:3)*p1';
                p1_d2=-critical_plane_norm_vector(i,1:3)*p1';       
                p2=renovation_effective_waypaths{i}(n,4:6);
                p2_d1=-intersect_plane_norm_vector(i,1:3)*p2';
                p2_d2=-critical_plane_norm_vector(i,1:3)*p2';
                
                flag1=0; flag2=0;
                if j==1
                    if p1_d1<=intersect_plane_d{i}(1,j) && p2_d1<=intersect_plane_d{i}(1,j)
                        flag1=1;
                    end
                end
                if j~=size(intersect_plane_d{i},2)+1
                    if j~=1
                        if p1_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p1_d1<=intersect_plane_d{i}(1,j)+0.01 && p2_d1>=intersect_plane_d{i}(1,j-1)-0.01 && p2_d1<=intersect_plane_d{i}(1,j)+0.01
                            flag1=1;
                        end
                    end
                end
                if j==size(intersect_plane_d{i},2)+1
                    if p1_d1>=intersect_plane_d{i}(1,j-1) && p2_d1>=intersect_plane_d{i}(1,j-1)
                        flag1=1;
                    end
                end
                
                if k==1
                    if p1_d2<=intersect_plane2_d{i}(1,k) && p2_d2<=intersect_plane2_d{i}(1,k)
                        flag2=1;
                    end
                end
                if  k~=size(intersect_plane2_d{i},2)+1
                    if k~=1
                        if p1_d2>=intersect_plane2_d{i}(1,k-1)-0.01 && p1_d2<=intersect_plane2_d{i}(1,k)+0.01 && p2_d2>=intersect_plane2_d{i}(1,k-1)-0.01 && p2_d2<=intersect_plane2_d{i}(1,k)+0.01
                            flag2=1;
                        end
                    end
                end
                if k==size(intersect_plane2_d{i},2)+1
                    if p1_d2>=intersect_plane2_d{i}(1,k-1) && p2_d2>=intersect_plane2_d{i}(1,k-1)
                        flag2=1;
                    end
                end
                % renovation_cells_waypaths
                if flag1==1 && flag2==1
                    renovation_cells_waypaths{i}{j}{k}(renovation_cells_edges_num,1:6)=renovation_effective_waypaths{i}(n,1:6);
                    renovation_cells_edges_num=renovation_cells_edges_num+1;
                end
                
            end
        end
    end
end

%% renovation_cells_clustering_waypaths should be generated from renovation_cells_waypaths 
%% input:renovation_cells_waypaths
%% output:renovation_cells_clustering_waypaths
for i=1:1:size(renovation_cells_waypaths,2)
    for j=1:1:size(renovation_cells_waypaths{i},2)
        for k=1:1:size(renovation_cells_waypaths{i}{j},2)
            a=renovation_cells_waypaths{i}{j}{k};
            b=path_clustering(a);
            renovation_cells_clustering_waypaths{i}{j}{k}=b;
        end
    end
end


%% add orientations into renovation path waypoints 
theta_x=-pi/2;
theta_z=pi/2;

for i=1:1:size(renovation_planes_norm_vector,2)
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
end 


for i=1:1:size(renovation_cells_clustering_waypaths,2)
    for j=1:1:size(renovation_cells_clustering_waypaths{i},2)
        for k=1:1:size(renovation_cells_clustering_waypaths{i}{j},2)
            for m=1:1:size(renovation_cells_clustering_waypaths{i}{j}{k},1)+1
                if m<=size(renovation_cells_clustering_waypaths{i}{j}{k},1)
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1:3)=renovation_cells_clustering_waypaths{i}{j}{k}(m,1:3)*0.001;
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,4:6)=[theta_x,theta_y(i),theta_z];
                else
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,1:3)=renovation_cells_clustering_waypaths{i}{j}{k}(m-1,4:6)*0.001;
                    renovation_cells_waypioints_onwaypath{i}{j}{k}(m,4:6)=[theta_x,theta_y(i),theta_z];
                end
            end
        end
    end
end

end



%% the above program should be transformed 
%% input: renovationcells_pathswaypoints_classified{plane_num}{h_num}{v_num}(waypath_num,1:6)
%% input: renovation_cells_manipulatorbase_positions{plane_num}{h_num}{v_num}(1,1:6)
%% input: manipulator_endeffector_positions{plane_num}{h_num}{v_num}(waypath_num,1:6)
%% input: renovation_cells_edges_classified{plane_num}{h_num}{v_num}()

% renovationcells_pathswaypoints_classified{plane_num}{h_num}{v_num}(points_num,1:6)
% renovation_cells_manipulatorbase_positions{plane_num}{h_num}{v_num}(1,1:6)
% manipulator_endeffector_positions{plane_num}{h_num}{v_num}(points_num,1:6)
% renovation_cells_edges_classified{plane_num}{h_num}{v_num}(edges_num,1:6)
% renovation_cells_paths_classified{plane_num}{h_num}{v_num}(paths_num,1:6)




