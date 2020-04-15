function [room_renovation_pathcell_points,room_renovation_pathcell_midpoints_flag,pathcell_minmax_criticalpoint,critical_plane_norm_vector]=room_renovation_pathcells_generation(renovation_planes_edge_cell,renovation_planes_norm_vector,renovation_plane_triangle_edges,path_interval,pathcell_interval,distance_renovationpathpoints_planeboundary)

%% obtain intersection plane norm vector which is normal to renovation plane
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
%% obtain renovation_planes_edge_points
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
%% obtain intersect plane parameter:d 
% input: path_interval
for i=1:1:size(intersect_plane_dmin_max,2)
    dmin=intersect_plane_dmin_max{i}(1,1);
    dmax=intersect_plane_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax-dmin)/path_interval)+0.5;
    for j=1:1:plane_num 
        intersect_plane_d{i}(1,j)=dmin+(j-0.5)*path_interval;
    end
end
%% obtain intersection points between renovation planes edge and renovation intersect plane
% input: intersect_plane_d,intesect_plane_norm_vector,renovation_planes_edge_cell
% output: room_renovation_path_points
for i=1:1:size(intersect_plane_d,2)
    for j=1:1:size(intersect_plane_d{i},2)
        a=intersect_plane_norm_vector(i,1);
        b=intersect_plane_norm_vector(i,2);
        c=intersect_plane_norm_vector(i,3);
        d=intersect_plane_d{i}(1,j);
        path_points_num=1;
        for k=1:1:size(renovation_planes_edge_cell{i},1)
            p1=renovation_planes_edge_cell{i}(k,1:3);
            p2=renovation_planes_edge_cell{i}(k,4:6);
            v=p2-p1;
            if (a*v(1)+b*v(2)+c*v(3))~=0
               t=-(a*p1(1)+b*p1(2)+c*p1(3)+d)/(a*v(1)+b*v(2)+c*v(3));
               if t>=0 && t<=1
                   room_renovation_path_points{i}{j}(path_points_num,1)=p1(1)+v(1)*t;
                   room_renovation_path_points{i}{j}(path_points_num,2)=p1(2)+v(2)*t;
                   room_renovation_path_points{i}{j}(path_points_num,3)=p1(3)+v(3)*t;
                   path_points_num=path_points_num+1;
               end
            end
        end
    end
end
%% choose effective renovation paths on renovation planes
% input: room_renovation_path_points,
% input: room_renovation_effective_paths
% firstly initial path points should be classified 
% secondly midpoint of path segments should be computed
% thirdly midpoint of path segments is applied to test whether path is effective 
for i=1:1:size(room_renovation_path_points,2)
    for j=1:1:size(room_renovation_path_points{i},2)
        path_points=room_renovation_path_points{i}{j};
        path_points=sortrows(path_points,1);
        path_points=sortrows(path_points,2);
        path_points=sortrows(path_points,3);
        room_renovation_path_points{i}{j}=path_points;
    end
end
% the above operation has been solved the problem of points classification
% on one renovation path
for i=1:1:size(room_renovation_path_points,2)
    for j=1:1:size(room_renovation_path_points{i},2)
        for k=1:1:size(room_renovation_path_points{i}{j},1)-1
            room_renovation_path_midpoints{i}{j}(k,1)=(room_renovation_path_points{i}{j}(k,1)+room_renovation_path_points{i}{j}(k+1,1))/2;            
            room_renovation_path_midpoints{i}{j}(k,2)=(room_renovation_path_points{i}{j}(k,2)+room_renovation_path_points{i}{j}(k+1,2))/2;            
            room_renovation_path_midpoints{i}{j}(k,3)=(room_renovation_path_points{i}{j}(k,3)+room_renovation_path_points{i}{j}(k+1,3))/2;            
        end
    end
end

for i=1:1:size(renovation_planes_norm_vector,2)
    % size(renovation_plane_triangle_edges{i},1)/3;
    for j=1:1:size(renovation_plane_triangle_edges{i},1)/3
        room_triangle_planes{i}(j,1:3)=renovation_planes_norm_vector{i}(1,1:3);
        room_triangle_planes{i}(j,4)=-renovation_planes_norm_vector{i}(1,1:3)*renovation_plane_triangle_edges{i}(3*j,1:3)';
    end
end
for i=1:1:size(room_triangle_planes,2)
    for j=1:1:size(room_triangle_planes{i},1)
        for k=1:1:3
            delta1=renovation_plane_triangle_edges{i}(3*(j-1)+k,4:6)-renovation_plane_triangle_edges{i}(3*(j-1)+k,1:3);
            delta2=renovation_planes_norm_vector{i}(1,1:3);
            renovation_triangle_boundary_plane{i}(3*(j-1)+k,1)=delta1(2)*delta2(3)-delta2(2)*delta1(3);
            renovation_triangle_boundary_plane{i}(3*(j-1)+k,2)=delta1(3)*delta2(1)-delta2(3)*delta1(1);
            renovation_triangle_boundary_plane{i}(3*(j-1)+k,3)=delta1(1)*delta2(2)-delta2(1)*delta1(2);
            renovation_triangle_boundary_plane{i}(3*(j-1)+k,4)=-renovation_triangle_boundary_plane{i}(3*(j-1)+k,1:3)*renovation_plane_triangle_edges{i}(3*(j-1)+k,1:3)';
        end
    end
end
for i=1:1:size(room_triangle_planes,2)
    for j=1:1:size(room_triangle_planes{i},1)
        renovation_triangle_centroid{i}(j,1)=sum(renovation_plane_triangle_edges{i}(3*j-2:3*j,1)+renovation_plane_triangle_edges{i}(3*j-2:3*j,4))/6;
        renovation_triangle_centroid{i}(j,2)=sum(renovation_plane_triangle_edges{i}(3*j-2:3*j,2)+renovation_plane_triangle_edges{i}(3*j-2:3*j,5))/6;
        renovation_triangle_centroid{i}(j,3)=sum(renovation_plane_triangle_edges{i}(3*j-2:3*j,3)+renovation_plane_triangle_edges{i}(3*j-2:3*j,6))/6;
        for k=1:1:3
            if renovation_triangle_centroid{i}(j,1:3)*renovation_triangle_boundary_plane{i}(3*(j-1)+k,1:3)'+renovation_triangle_boundary_plane{i}(3*(j-1)+k,4)<=0
                renovation_triangle_centriod_flag{i}(j,k)=0;
            else
                renovation_triangle_centriod_flag{i}(j,k)=1;
            end
        end
    end
end
for i=1:1:size(room_renovation_path_midpoints,2)
    for j=1:1:size(room_renovation_path_midpoints{i},2)
        for k=1:1:size(room_renovation_path_midpoints{i}{j},1)
            for m=1:1:size(room_triangle_planes{i},1)
                for n=1:1:3
                    sumt=0;
                    for s=1:1:3
                        sumt=sumt+room_renovation_path_midpoints{i}{j}(k,s)*renovation_triangle_boundary_plane{i}(3*(m-1)+n,s);
                    end
                    if sumt+renovation_triangle_boundary_plane{i}(3*(m-1)+n,4)<=0
                        room_triangle_intersect_point_flag{i}{j}(k,m,n)=0;
                    else
                        room_triangle_intersect_point_flag{i}{j}(k,m,n)=1;
                    end
                end
            end
        end
    end
end
for i=1:1:size(room_renovation_path_midpoints,2)
    for j=1:1:size(room_renovation_path_midpoints{i},2)
        for k=1:1:size(room_renovation_path_midpoints{i}{j},1)
            count=0;
            for m=1:1:size(room_triangle_planes{i},1)
                if room_triangle_intersect_point_flag{i}{j}(k,m,1)==renovation_triangle_centriod_flag{i}(m,1) && room_triangle_intersect_point_flag{i}{j}(k,m,2)==renovation_triangle_centriod_flag{i}(m,2) && room_triangle_intersect_point_flag{i}{j}(k,m,3)==renovation_triangle_centriod_flag{i}(m,3)
                    count=1;
                end
            end
            if count==1
                room_renovation_path_midpoints_flag{i}{j}(1,k)=1;
            else
                room_renovation_path_midpoints_flag{i}{j}(1,k)=0;
            end
        end
    end
end
%% the distance between room renovation path points and room renovation plane boundaries should be kept
%% input: distance_renovationpathpoints_planeboundary
%% this part is ignored now.
for i=1:1:size(room_renovation_path_points,2)
    for j=1:1:size(room_renovation_path_points{i},2)
        p1=room_renovation_path_points{i}{j}(1,1:3);
        p2=room_renovation_path_points{i}{j}(2,1:3);
        path_vector=p2-p1;
        distance_vector=distance_renovationpathpoints_planeboundary/norm(path_vector)*path_vector;
        for k=1:1:size(room_renovation_path_points{i}{j},1)
            room_renovation_path_points_withdistance{i}{j}(k,1:3)=room_renovation_path_points{i}{j}(k,1:3)+distance_vector;
            room_renovation_path_points_withdistance{i}{j}(k,4:6)=room_renovation_path_points{i}{j}(k,1:3)-distance_vector;
        end
    end
end
for i=1:1:size(room_renovation_path_points_withdistance,2)
    for j=1:1:size(room_renovation_path_points_withdistance{i},2)
        for k=1:1:size(room_renovation_path_points_withdistance{i}{j},1)
            p1=room_renovation_path_points_withdistance{i}{j}(k,1:3);
            p2=room_renovation_path_points_withdistance{i}{j}(k,4:6);
        end
    end
end

%% divide renovation paths into path cells 
% here a simple strategy is applid: equal width decomposition method 
% input: pathcell_interval, path_interval

pathcell_pathnum=pathcell_interval/path_interval;
for i=1:1:size(room_renovation_path_points,2)
    for j=1:1:size(room_renovation_path_points{i},2)
        if mod(j,pathcell_pathnum)~=0
            m=ceil(j/pathcell_pathnum);
            n=mod(j,pathcell_pathnum);
        else
            m=j/pathcell_pathnum;
            n=pathcell_pathnum;
        end
        room_renovation_pathcell_points{i}{m}{n}=room_renovation_path_points{i}{j};
        room_renovation_pathcell_midpoints_flag{i}{m}{n}=room_renovation_path_midpoints_flag{i}{j};
    end
end

%% obtain critical points for path cells
for i=1:1:size(room_renovation_pathcell_points,2)
% for i=8:1:8
    for j=1:1:size(room_renovation_pathcell_points{i},2)
        k=1;
        pathcell_points=[];
        for m=1:1:size(room_renovation_pathcell_points{i}{j},2)
            for n=1:1:size(room_renovation_pathcell_points{i}{j}{m},1)
                pathcell_points(k,1:3)=room_renovation_pathcell_points{i}{j}{m}(n,1:3);
                k=k+1;
            end
        end
        pathcell_points=sortrows(pathcell_points,1);
        pathcell_points=sortrows(pathcell_points,2);
        pathcell_points=sortrows(pathcell_points,3);

        pathcell_minmax_criticalpoint{i}{j}(1,1:3)=pathcell_points(1,1:3);
        pathcell_minmax_criticalpoint{i}{j}(2,1:3)=pathcell_points(size(pathcell_points,1),1:3);
    end
end

end
% 




