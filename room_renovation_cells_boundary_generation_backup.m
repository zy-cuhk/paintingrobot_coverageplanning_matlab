function [renovation_cells_edges_classified,renovation_cells_paths_classified,renovationcells_pathswaypoints_classified,renovation_cells_manipulatorbase_positions,manipulator_endeffector_positions]=room_renovation_cells_boundary_generation(room_verticalplanes_edge_cell,renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_planes_norm_vector,length_interval,width_interval,path_distance)

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

%% obtain vertical intersection points, the output: vertical_intersection_points
% input: intersect_plane_d,intesect_plane_norm_vector,renovation_planes_edge_cell
% output: room_renovation_path_points

for i=1:1:size(intersect_plane_dmin_max,2)
    dmin=intersect_plane_dmin_max{i}(1,1);
    dmax=intersect_plane_dmin_max{i}(1,2);
    plane_num=floor(abs(dmax-dmin)/length_interval);
    for j=1:1:plane_num 
        intersect_plane_d{i}(1,j)=dmin+j*length_interval;
    end
end

renovation_planes_edge_cell01=renovation_planes_edge_cell;
for i=1:1:size(intersect_plane_d,2)
    for j=1:1:size(intersect_plane_d{i},2)
        a=intersect_plane_norm_vector(i,1);
        b=intersect_plane_norm_vector(i,2);
        c=intersect_plane_norm_vector(i,3);
        d=intersect_plane_d{i}(1,j);
        path_points_num=1;
        vector=[];
        for k=1:1:size(renovation_planes_edge_cell01{i},1)
            p1=renovation_planes_edge_cell01{i}(k,1:3);
            p2=renovation_planes_edge_cell01{i}(k,4:6);
            v=p2-p1;
            if (a*v(1)+b*v(2)+c*v(3))~=0
               t=-(a*p1(1)+b*p1(2)+c*p1(3)+d)/(a*v(1)+b*v(2)+c*v(3));
               if t>=0 && t<=1
                   vertical_intersection_points{i}{j}(path_points_num,1)=p1(1)+v(1)*t;
                   vertical_intersection_points{i}{j}(path_points_num,2)=p1(2)+v(2)*t;
                   vertical_intersection_points{i}{j}(path_points_num,3)=p1(3)+v(3)*t;
                   path_points_num=path_points_num+1;
               end
            end
        end
        vector=vertical_intersection_points{i}{j};
        vector=sortrows(vector,1);
        vector=sortrows(vector,2);
        vector=sortrows(vector,3);
        edge_cell{i}(j,1:3)=vector(1,1:3);
        edge_cell{i}(j,4:6)=vector(end,1:3);
    end
end
for i=1:1:size(edge_cell,2)
    n0=size(edge_cell{i},1);
    for j=1:1:size(renovation_planes_edge_cell01{i},1)
        edge_cell{i}(n0+j,1:6)=renovation_planes_edge_cell01{i}(j,1:6);
    end
end



%% put vertical_intersection_points into intersection_points

for i=1:1:size(vertical_intersection_points,2)
    intersection_points_num=1;
    for j=1:1:size(vertical_intersection_points{i},2)
        for k=1:1:size(vertical_intersection_points{i}{j},1)
            intersection_points{i}(intersection_points_num,1:3)=vertical_intersection_points{i}{j}(k,1:3);
            intersection_points_num=intersection_points_num+1;
        end
    end
end
for i=1:1:size(renovation_planes_edge_points,2)
    intersection_points_num=size(intersection_points{i},1);
    for j=1:1:size(renovation_planes_edge_points{i},1)
        intersection_points{i}(intersection_points_num+j,1:3)=renovation_planes_edge_points{i}(j,1:3);
    end
end




%% obtain horizontal intersection points, the output:horizontal_intersection_points
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

for i=1:1:size(intersect_plane2_d,2)
%% for i=4:1:4
    n0=size(edge_cell{i},1);
    for j=1:1:size(intersect_plane2_d{i},2)
        a=critical_plane_norm_vector(i,1);
        b=critical_plane_norm_vector(i,2);
        c=critical_plane_norm_vector(i,3);
        d=intersect_plane2_d{i}(1,j);
        path_points_num=1;
        vector=[];
        for k=1:1:size(edge_cell{i},1)
            p1=edge_cell{i}(k,1:3);
            p2=edge_cell{i}(k,4:6);
            v=p2-p1;
            if (a*v(1)+b*v(2)+c*v(3))~=0
               t=-(a*p1(1)+b*p1(2)+c*p1(3)+d)/(a*v(1)+b*v(2)+c*v(3));
               if t>=0 && t<=1
                   horizontal_intersection_points{i}{j}(path_points_num,1)=p1(1)+v(1)*t;
                   horizontal_intersection_points{i}{j}(path_points_num,2)=p1(2)+v(2)*t;
                   horizontal_intersection_points{i}{j}(path_points_num,3)=p1(3)+v(3)*t;
                   path_points_num=path_points_num+1;
               end
            end
        end
        vector=horizontal_intersection_points{i}{j};
        vector=sortrows(vector,1);
        vector=sortrows(vector,2);
        % vector=sortrows(vector,3);
        edge_cell{i}(n0+j,1:3)=vector(1,1:3);
        edge_cell{i}(n0+j,4:6)=vector(end,1:3);
    end
end

%% put horizontal_intersection_points into intersection_points
for i=1:1:size(horizontal_intersection_points,2)
    intersection_points_num=size(intersection_points{i},1);
    m=1;
    for j=1:1:size(horizontal_intersection_points{i},2)
        for k=1:1:size(horizontal_intersection_points{i}{j},1)
            intersection_points{i}(intersection_points_num+m,1:3)=horizontal_intersection_points{i}{j}(k,1:3);
            m=m+1;
        end
    end
end


%% gather intersection points into edges, and the edges should belong to renovation cells in next a few steps
for i=1:1:size(edge_cell,2)
    for j=1:1:size(edge_cell{i},1)
        p1=edge_cell{i}(j,1:3);
        p2=edge_cell{i}(j,4:6);
        % edge_points{i}{j}(1,1:3)=edge_cell{i}(j,1:3);
        % edge_points{i}{j}(2,1:3)=edge_cell{i}(j,4:6);
        num=1;
        for k=1:1:size(intersection_points{i},1)
            xlenda=-1; ylenda=-1; zlenda=-1;
            flag1=0; flag2=0; flag3=0;
            p3=intersection_points{i}(k,1:3);
            if abs(p1(1)-p2(1))<=0.01
                if abs(p3(1)-p1(1))<0.01
                    flag1=1;
                end
            else
                xlenda=(p3(1)-p1(1))/(p2(1)-p1(1));
            end
            if abs(p1(2)-p2(2))<=0.01
                if abs(p3(2)-p1(2))<=0.01
                    flag2=1;
                end
            else
                ylenda=(p3(2)-p1(2))/(p2(2)-p1(2));
            end
            if abs(p1(3)-p2(3))<=0.01
                if abs(p3(3)-p1(3))<=0.01
                    flag3=1;
                end
            else
                zlenda=(p3(3)-p1(3))/(p2(3)-p1(3));
            end
            
            if flag1==1 && flag2==1 && flag3==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            
            if xlenda>=-0.01 && xlenda<=1.01 && flag2==1 && flag3==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            if ylenda>=-0.01 && ylenda<=1.01 && flag1==1 && flag3==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            if zlenda>=-0.01 && zlenda<=1.01 && flag1==1 && flag2==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            
            if abs(ylenda-zlenda)<=0.01 && ylenda>=-0.01 && ylenda<=1.01 && flag1==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            if abs(xlenda-zlenda)<=0.01 && xlenda>=-0.01 && xlenda<=1.01 && flag2==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end
            if abs(xlenda-ylenda)<=0.01 && xlenda>=-0.01 && xlenda<=1.01 && flag3==1
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end

            if abs(xlenda-ylenda)<=0.01 && abs(xlenda-zlenda)<=0.01 && abs(ylenda-zlenda)<=0.01 && xlenda>=-0.01 && xlenda<=1.01
                edge_points{i}{j}(num,1:3)=intersection_points{i}(k,1:3);
                num=num+1;
            end

        end
        edge_points{i}{j}=sortrows(edge_points{i}{j},1);
        edge_points{i}{j}=sortrows(edge_points{i}{j},2);
        edge_points{i}{j}=sortrows(edge_points{i}{j},3);
        for m=1:1:size(edge_points{i}{j},1)-1
            edge_points_segment{i}{j}(m,1:3)=edge_points{i}{j}(m,1:3);
            edge_points_segment{i}{j}(m,4:6)=edge_points{i}{j}(m+1,1:3);
        end
    end
end




%% gather renovation cells edges into renovation cells, the output is:renovation_cells_edges
% input: edge_points_segment
% input: intersect_plane_norm_vector,intersect_plane_d
% input: critical_plane_norm_vector,intersect_plane2_d
% output: renovation_cells_edges

ts=1;
for i=1:1:size(intersect_plane_d,2)
% for i=1:1:1
    cell_num=1;
    for j=1:1:size(intersect_plane_d{i},2)+1
       for k=1:1:size(intersect_plane2_d{i},2)+1
            
            renovation_cells_edges_num=1;
            for m=1:1:size(edge_points_segment{i},2)
                for n=1:1:size(edge_points_segment{i}{m},1)
                    p1=edge_points_segment{i}{m}(n,1:3);

                    p1_d1=-intersect_plane_norm_vector(i,1:3)*p1';
                    p1_d2=-critical_plane_norm_vector(i,1:3)*p1';
                    
                    p2=edge_points_segment{i}{m}(n,4:6);
                    
                    p2_d1=-intersect_plane_norm_vector(i,1:3)*p2';
                    p2_d2=-critical_plane_norm_vector(i,1:3)*p2';
                    
                    if i==1 && cell_num==1
                        pd(ts,1)=p1_d1;
                        pd(ts,2)=p2_d1;
                        pd(ts,3)=p1_d2;
                        pd(ts,4)=p2_d2;
                        ts=ts+1;
                    end
                    
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
                    if flag1==1 && flag2==1
                        renovation_cells_edges{i}{cell_num}(renovation_cells_edges_num,1:6)=edge_points_segment{i}{m}(n,1:6);
                        renovation_cells_edges_num=renovation_cells_edges_num+1;
                    end
                    
                end
            end
            cell_num=cell_num+1;
        end
    end
end

pd=sortrows(pd);

%% delete painting cells without any thing inside it
renovation_cells_edges01=renovation_cells_edges;
renovation_cells_edges={};
for i=1:1:size(renovation_cells_edges01,2)
    m=1;
    for j=1:1:size(renovation_cells_edges01{i},2)
        if size(renovation_cells_edges01{i}{j},1)~=0
            renovation_cells_edges{i}{m}=renovation_cells_edges01{i}{j};
            m=m+1;
        end
    end
end


%% generate paths in each painting cells, that is: renovation_cells_paths
for i=1:1:size(renovation_cells_edges,2)
    for j=1:1:size(renovation_cells_edges{i},2)
        for k=1:1:size(renovation_cells_edges{i}{j},1)
            renovation_cells_edges_points{i}{j}(2*k-1,1:3)=renovation_cells_edges{i}{j}(k,1:3);
            renovation_cells_edges_points{i}{j}(2*k,1:3)=renovation_cells_edges{i}{j}(k,4:6);
        end
        renovation_cells_edges_points{i}{j}=unique(renovation_cells_edges_points{i}{j},'rows');
    end
end
for i=1:1:size(renovation_cells_edges_points,2)
    for j=1:1:size(renovation_cells_edges_points{i},2)
        for k=1:1:size(renovation_cells_edges_points{i}{j},1)
            renovation_cells_intersect_plane_d_candidate{i}{j}(1,k)=-critical_plane_norm_vector(i,1:3)*renovation_cells_edges_points{i}{j}(k,1:3)';
        end
        renovation_cells_intersect_plane_dmin_max{i}{j}(1,1)=min(renovation_cells_intersect_plane_d_candidate{i}{j});
        renovation_cells_intersect_plane_dmin_max{i}{j}(1,2)=max(renovation_cells_intersect_plane_d_candidate{i}{j});
    end
end



for i=1:1:size(renovation_cells_intersect_plane_dmin_max,2)
    for j=1:1:size(renovation_cells_intersect_plane_dmin_max{i},2)
        dmin=renovation_cells_intersect_plane_dmin_max{i}{j}(1,1);
        dmax=renovation_cells_intersect_plane_dmin_max{i}{j}(1,2);
        plane_num=floor(abs(dmax-dmin)/path_distance);
        for k=1:1:plane_num
            renovation_cells_intersect_plane_d{i}{j}(1,k)=dmin+(k-0.5)*path_distance;
        end
    end
end

for i=1:1:size(renovation_cells_intersect_plane_d,2)
    a=critical_plane_norm_vector(i,1);
    b=critical_plane_norm_vector(i,2);
    c=critical_plane_norm_vector(i,3);
    for m=1:1:size(renovation_cells_edges{i},2)
        for k=1:1:size(renovation_cells_intersect_plane_d{i}{m},2)
            d=renovation_cells_intersect_plane_d{i}{m}(1,k);
            path_points_num=1;
            for n=1:1:size(renovation_cells_edges{i}{m},1)
                p1=renovation_cells_edges{i}{m}(n,1:3);
                p2=renovation_cells_edges{i}{m}(n,4:6);
                v=p2-p1;
                vector=[];
                if (a*v(1)+b*v(2)+c*v(3))~=0
                    t=-(a*p1(1)+b*p1(2)+c*p1(3)+d)/(a*v(1)+b*v(2)+c*v(3));
                    if t>=0 && t<=1
                        renovation_cells_intersection_points{i}{m}{k}(path_points_num,1)=p1(1)+v(1)*t;
                        renovation_cells_intersection_points{i}{m}{k}(path_points_num,2)=p1(2)+v(2)*t;
                        renovation_cells_intersection_points{i}{m}{k}(path_points_num,3)=p1(3)+v(3)*t;
                        path_points_num=path_points_num+1;
                    end
                end
            end
            vector=renovation_cells_intersection_points{i}{m}{k};
            vector=sortrows(vector,1);
            vector=sortrows(vector,2);
            vector=sortrows(vector,3);
            renovation_cells_paths{i}{m}(k,1:3)=vector(1,1:3);
            renovation_cells_paths{i}{m}(k,4:6)=vector(end,1:3);
        end
    end
end

%% classify renovation cells 
% for i=1:1:1
for i=1:1:size(renovation_cells_edges,2)
    for j=1:1:size(intersect_plane_d{i},2)+1
        a=intersect_plane_norm_vector(i,1);
        b=intersect_plane_norm_vector(i,2);
        c=intersect_plane_norm_vector(i,3);
        num=1;
        for m=1:1:size(renovation_cells_edges{i},2)
            flag=0;
            for n=1:1:size(renovation_cells_edges{i}{m},1)
                for s=1:1:2
                    x0=renovation_cells_edges{i}{m}(n,3*s-2);
                    y0=renovation_cells_edges{i}{m}(n,3*s-1);
                    z0=renovation_cells_edges{i}{m}(n,3*s);
                    d=-(a*x0+b*y0+c*z0);
                    if j==1
                        if d-intersect_plane_d{i}(1,j)>0.01
                            flag=1;
                        end
                    end
                    if j==size(intersect_plane_d{i},2)+1
                        if d-intersect_plane_d{i}(1,j-1)<-0.01
                            flag=1;
                        end
                    end
                    if j~=size(intersect_plane_d{i},2)+1
                        if j~=1
                            if d-intersect_plane_d{i}(1,j-1)<-0.01 || d-intersect_plane_d{i}(1,j)>0.01
                                flag=1;
                            end
                        end
                    end
                end
            end
            if flag==0
                renovation_cells_edges_classified{i}{j}{num}=renovation_cells_edges{i}{m};
                renovation_cells_paths_classified{i}{j}{num}=renovation_cells_paths{i}{m};
                num=num+1;
            end
        end
    end
end



%% generate renovation path waypoints with orientations in each cells 
% input: renovation_planes_norm_vector
% input: renovation_cells_paths_classified{plane_num}{horizontal_num}{vertical_num}(paths_num,1:6)
% output: renovation_cells_pathswaypoints_classified{plane_num}{horizontal_num}{vertical_num}(waypoints_num,1:6)

theta_x=0;
theta_y=pi/2;

for i=1:1:size(renovation_planes_norm_vector,2)
    nx=renovation_planes_norm_vector{i}(1,1);
    ny=renovation_planes_norm_vector{i}(1,2);
    nz=renovation_planes_norm_vector{i}(1,3);
    cos_theta_y=nx;
    sin_theta_y=ny;
    if sin_theta_y>=0
        if cos_theta_y>=0
            theta_z(i)=asin(sin_theta_y);
        else
            theta_z(i)=pi-asin(sin_theta_y);
        end
    else
        if cos_theta_y>=0
            theta_z(i)=2*pi-asin(abs(sin_theta_y));
        else
            theta_z(i)=pi+asin(abs(sin_theta_y));
        end
    end
end 

for i=1:1:size(renovation_cells_paths_classified,2)
    for j=1:1:size(renovation_cells_paths_classified{i},2)
        for k=1:1:size(renovation_cells_paths_classified{i}{j},2)
            paths_points=[];
            for m=1:1:size(renovation_cells_paths_classified{i}{j}{k},1)
                paths_points(2*m-1,1:3)=renovation_cells_paths_classified{i}{j}{k}(m,1:3)*0.001;
                paths_points(2*m-1,4:6)=[theta_z(i),theta_y,theta_x];
                paths_points(2*m,1:3)=renovation_cells_paths_classified{i}{j}{k}(m,4:6)*0.001;
                paths_points(2*m,4:6)=[theta_z(i),theta_y,theta_x];
            end
            % path_points=unique(path_points,'rows');
            renovationcells_pathswaypoints_classified{i}{j}{k}=paths_points;
        end
    end
end

size(renovation_cells_edges_classified{1}{1}{1})
size(renovation_cells_paths_classified{1}{1}{1})
size(renovationcells_pathswaypoints_classified{1}{1}{1})

%% obtain manipulator base positions with orientations for each renovation cell
% input: renovation_cells_paths_classified{}{}{}()
% output: renovationcells_manipulatorbase_positions
for i=1:1:size(renovation_cells_edges_classified,2)
    for j=1:1:size(renovation_cells_edges_classified{i},2)
        n=1; xy_vector=[];
        for k=1:1:size(renovation_cells_edges_classified{i}{j},2)
            for m=1:1:size(renovation_cells_edges_classified{i}{j}{k},1)
                xy_vector(n,1)=renovation_cells_edges_classified{i}{j}{k}(m,1);
                xy_vector(n,2)=renovation_cells_edges_classified{i}{j}{k}(m,2);
                n=n+1;
            end
        end
        xy_vector=unique(xy_vector,'rows');
        xy_vector=sortrows(xy_vector);
        renovationcells_horizontalposition{i}(2*j-1,1:2)=xy_vector(1,1:2);
        renovationcells_horizontalposition{i}(2*j,1:2)=xy_vector(end,1:2);
    end
end
for i=1:1:size(renovationcells_horizontalposition,2)
    for j=1:1:size(renovationcells_horizontalposition{i},1)/2
        x0=renovationcells_horizontalposition{i}(2*j-1,1);
        y0=renovationcells_horizontalposition{i}(2*j-1,2);
        x1=renovationcells_horizontalposition{i}(2*j,1);
        y1=renovationcells_horizontalposition{i}(2*j,2);
        distance=sqrt((x0-x1)^2+(y0-y1)^2);
        renovationcells_horizontal_midposition{i}(j,1)=(renovationcells_horizontalposition{i}(2*j-1,1)+renovationcells_horizontalposition{i}(2*j,1))/2;
        renovationcells_horizontal_midposition{i}(j,2)=(renovationcells_horizontalposition{i}(2*j-1,2)+renovationcells_horizontalposition{i}(2*j,2))/2;
    end
end
for i=1:1:size(renovationcells_horizontal_midposition,2)
    for j=1:1:size(renovationcells_horizontal_midposition{i},1)
        x0=renovationcells_horizontalposition{i}(2*j-1,1);
        y0=renovationcells_horizontalposition{i}(2*j-1,2);
        x1=renovationcells_horizontalposition{i}(2*j,1);
        y1=renovationcells_horizontalposition{i}(2*j,2);
        
        a1=y1-y0;
        b1=x0-x1;
        a2=b1;
        b2=-a1;
        c2=-(a2*renovationcells_horizontal_midposition{i}(j,1)+b2*renovationcells_horizontal_midposition{i}(j,2));
        
        intersection_line{i}(j,1)=a2;
        intersection_line{i}(j,2)=b2;
        intersection_line{i}(j,3)=c2;
        
    end
end
for i=1:1:size(intersection_line,2)
    for j=1:1:size(intersection_line{i},1)
        a1=intersection_line{i}(j,1);
        b1=intersection_line{i}(j,2);
        c1=intersection_line{i}(j,3);
        a2=renovation_manipulatorbase_planes{i}(1,1);
        b2=renovation_manipulatorbase_planes{i}(1,2);
        c2=renovation_manipulatorbase_planes{i}(1,3);
        
        renovation_horizontalcells_manipulatorbase_points{i}(j,1)=(c2*b1-c1*b2)/(a1*b2-a2*b1);
        renovation_horizontalcells_manipulatorbase_points{i}(j,2)=(c1*a2-c2*a1)/(a1*b2-a2*b1);
        renovation_horizontalcells_manipulatorbase_points{i}(j,3)=0;
    end
end

for i=1:1:size(renovation_planes_norm_vector,2)
    nx=renovation_planes_norm_vector{i}(1,1);
    ny=renovation_planes_norm_vector{i}(1,2);
    nz=renovation_planes_norm_vector{i}(1,3);
    cos_theta_z=nx;
    sin_theta_z=ny;
    if sin_theta_z>=0
        if cos_theta_z>=0
            base_theta_z(i)=asin(sin_theta_z);
        else
            base_theta_z(i)=pi-asin(sin_theta_z);
        end
    else
        if cos_theta_z>=0
            base_theta_z(i)=2*pi-asin(abs(sin_theta_z));
        else
            base_theta_z(i)=pi+asin(abs(sin_theta_z));
        end
    end
end 
for i=1:1:size(renovation_cells_paths_classified,2)
    for j=1:1:size(renovation_cells_paths_classified{i},2)
        for k=1:1:size(renovation_cells_paths_classified{i}{j},2)
            num=size(renovation_cells_paths_classified{i}{j}{k},1);
            if mod(num,2)==1
                z1=renovation_cells_paths_classified{i}{j}{k}(1,3);
                z2=renovation_cells_paths_classified{i}{j}{k}(end,3);
                z_mean=(z1+z2)/2;
            else
                z1=renovation_cells_paths_classified{i}{j}{k}(1,3);
                z2=renovation_cells_paths_classified{i}{j}{k}(end,3);
                z_mean=(z1+z2)/2+path_distance/2;
            end
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,1)=renovation_horizontalcells_manipulatorbase_points{i}(j,1)*0.001;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,2)=renovation_horizontalcells_manipulatorbase_points{i}(j,2)*0.001;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,3)=z_mean*0.001;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,4)=base_theta_z(i);
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,5)=0;
            renovation_cells_manipulatorbase_positions{i}{j}{k}(1,6)=0;
        end
    end
end



%% compute manipulator end effector positions
% input: renovationcells_pathswaypoints_classified
% input: renovation_cells_manipulatorbase_positions
% output: manipulator_endeffector_positions{plane_num}{horizontal_num}{vertical_num}(pathwaypoints_num,1:6)


for i=1:1:size(renovation_cells_manipulatorbase_positions,2)
    for j=1:1:size(renovation_cells_manipulatorbase_positions{i},2)
        for k=1:1:size(renovation_cells_manipulatorbase_positions{i}{j},2)
% for i=1:1:1
%     for j=1:1:1
%         for k=1:1:1
            manipulatorbase_xyz=renovation_cells_manipulatorbase_positions{i}{j}{k}(1,1:3);
            manipulatorbase_rpy=renovation_cells_manipulatorbase_positions{i}{j}{k}(1,4:6);
            for m=1:1:size(renovationcells_pathswaypoints_classified{i}{j}{k},1)
            % for m=1:1:1
                renovationpathwaypoints_xyz=renovationcells_pathswaypoints_classified{i}{j}{k}(m,1:3);
                renovationpathwaypoints_rpy=renovationcells_pathswaypoints_classified{i}{j}{k}(m,4:6);
                rot_mat=rotz(renovationpathwaypoints_rpy(1))';
                renovationpathwaypoints_xyz-manipulatorbase_xyz;
                tran=rot_mat*(renovationpathwaypoints_xyz-manipulatorbase_xyz)';
                
                manipulatorpathwaypoints_xyzrpy=[];
                manipulatorpathwaypoints_xyzrpy(1,1)=tran(1);
                manipulatorpathwaypoints_xyzrpy(1,2)=tran(2);
                manipulatorpathwaypoints_xyzrpy(1,3)=tran(3);
                manipulatorpathwaypoints_xyzrpy(1,4)=0;
                manipulatorpathwaypoints_xyzrpy(1,5)=pi/2;
                manipulatorpathwaypoints_xyzrpy(1,6)=0;
                
                manipulator_endeffector_positions{i}{j}{k}(m,1:6)=manipulatorpathwaypoints_xyzrpy(1,1:6);
            end
        end
    end
end

for i=1:1:size(renovation_cells_edges_classified,2)
     for j=1:1:size(renovation_cells_edges_classified{i},2)
        for k=1:1:size(renovation_cells_edges_classified{i}{j},2)
            for n=1:1:size(renovation_cells_edges_classified{i}{j}{k},1)
                renovation_cells_edges_classified{i}{j}{k}(n,1:6)=renovation_cells_edges_classified{i}{j}{k}(n,1:6)*0.001;
            end
        end
    end
end



%% the above program should have two more modifications.
%% firstly, the function of determining whether the specific renovation cell is empty or not;
%% secondly, the computation of mobile platform positions is collision avoidance.


figure;
xlabel("x axis")
ylabel("y axis")
for i=1:1:size(renovationcells_horizontalposition,2)
    x=renovationcells_horizontalposition{i}(:,1);
    y=renovationcells_horizontalposition{i}(:,2);
    scatter(x,y,'*r');
    hold on;
end
for i=1:1:size(renovation_horizontalcells_manipulatorbase_points,2)
    x=renovation_horizontalcells_manipulatorbase_points{i}(:,1);
    y=renovation_horizontalcells_manipulatorbase_points{i}(:,2);
    scatter(x,y,'*b');
    hold on;
end
for i=1:1:size(renovationcells_horizontalposition,2)
    % for j=1:1:size(renovationcells_horizontalposition{i},1)
    x0=renovationcells_horizontalposition{i}(:,1);
    y0=renovationcells_horizontalposition{i}(:,2);
    line(x0,y0);
    % end
end

figure;
for i=1:1:size(renovation_cells_edges_classified,2)
% for i=1:1:1
    % for j=1:1:1
     for j=1:1:size(renovation_cells_edges_classified{i},2)
        for k=1:1:size(renovation_cells_edges_classified{i}{j},2)
            for n=1:1:size(renovation_cells_edges_classified{i}{j}{k},1)
                xlabel("x axis");
                ylabel("y axis");
                zlabel("z axis");
                title('3D model of interior surfaces framework','FontSize',24);
                x1=[renovation_cells_edges_classified{i}{j}{k}(n,1),renovation_cells_edges_classified{i}{j}{k}(n,4)];
                y1=[renovation_cells_edges_classified{i}{j}{k}(n,2),renovation_cells_edges_classified{i}{j}{k}(n,5)];
                z1=[renovation_cells_edges_classified{i}{j}{k}(n,3),renovation_cells_edges_classified{i}{j}{k}(n,6)];
                plot3(x1,y1,z1,'r','LineWidth',1);
                axis equal;
                view(-114,24);
                hold on;
            end
        end
    end
end

figure;
% for i=1:1:size(renovation_cells_edges_classified,2)
for i=1:1:1
    for j=1:1:1
     % for j=1:1:size(renovation_cells_edges_classified{i},2)
        for k=1:1:1
        % for k=1:1:size(renovation_cells_edges_classified{i}{j},2)
            for n=1:1:size(renovation_cells_edges_classified{i}{j}{k},1)
                xlabel("x axis");
                ylabel("y axis");
                zlabel("z axis");
                title('3D model of interior surfaces framework','FontSize',24);
                x1=[renovation_cells_edges_classified{i}{j}{k}(n,1),renovation_cells_edges_classified{i}{j}{k}(n,4)];
                y1=[renovation_cells_edges_classified{i}{j}{k}(n,2),renovation_cells_edges_classified{i}{j}{k}(n,5)];
                z1=[renovation_cells_edges_classified{i}{j}{k}(n,3),renovation_cells_edges_classified{i}{j}{k}(n,6)];
                plot3(x1,y1,z1,'r','LineWidth',1);
                axis equal;
                view(-114,24);
                hold on;
            end
        end
    end
end




% figure;
% xlabel("x axis")
% ylabel("y axis")
% for i=1:1:1
%     x=renovationcells_horizontalposition{i}(:,1);
%     y=renovationcells_horizontalposition{i}(:,2);
%     scatter(x,y,'*r');
%     hold on;
% end
% for i=1:1:1
%     x=renovation_horizontalcells_manipulatorbase_points{i}(:,1);
%     y=renovation_horizontalcells_manipulatorbase_points{i}(:,2);
%     scatter(x,y,'*b');
%     hold on;
% end
% for i=1:1:1
%     % for j=1:1:size(renovationcells_horizontalposition{i},1)
%     x0=renovationcells_horizontalposition{i}(:,1);
%     y0=renovationcells_horizontalposition{i}(:,2);
%     line(x0,y0);
%     % end
% end

% figure;
% for i=1:1:size(renovation_planes_edge_cell,2)
%         for k=1:1:size(renovation_planes_edge_cell{i},1)
%             xlabel("x axis");
%             ylabel("y axis");
%             zlabel("z axis");
%             title('3D model of interior surfaces framework','FontSize',24);
%             x1=[renovation_planes_edge_cell{i}(k,1),renovation_planes_edge_cell{i}(k,4)];
%             y1=[renovation_planes_edge_cell{i}(k,2),renovation_planes_edge_cell{i}(k,5)];
%             z1=[renovation_planes_edge_cell{i}(k,3),renovation_planes_edge_cell{i}(k,6)];
%             plot3(x1 ,y1,z1,'b','LineWidth',1);
%             axis equal;
%             view(-114,24);
%             hold on;
%         end
% end


end












% % for i=1:1:size(renovation_cells_paths,2)
% for i=4:1:4
%     for j=1:1:1
%     % for j=1:1:size(renovation_cells_paths{i},2)
%         for k=1:1:size(renovation_cells_paths{i}{j},1)
%             xlabel("x axis");
%             ylabel("y axis");
%             zlabel("z axis");
%             title('3D model of interior surfaces framework','FontSize',24);
%             x1=[renovation_cells_paths{i}{j}(k,1),renovation_cells_paths{i}{j}(k,4)];
%             y1=[renovation_cells_paths{i}{j}(k,2),renovation_cells_paths{i}{j}(k,5)];
%             z1=[renovation_cells_paths{i}{j}(k,3),renovation_cells_paths{i}{j}(k,6)];
%             plot3(x1,y1,z1,'r','LineWidth',1);
%             axis equal;
%             view(-114,24);
%             hold on;
%         end
%     end
% end

% figure;
% for i=1:1:size(intersection_points,2)
% for i=4:1:4
%     for j=1:1:size(renovation_cells_edges_points{i},2)
%         xarray=renovation_cells_edges_points{i}{j}(:,1);
%         yarray=renovation_cells_edges_points{i}{j}(:,2);
%         zarray=renovation_cells_edges_points{i}{j}(:,3);
%         scatter3(xarray,yarray,zarray);
%         hold on;
%     end
% end

% %% for i=1:1:size(edge_cell,2)
% for i=1:1:1
%     for j=1:1:size(edge_cell{i},1)
%         xlabel("x axis");
%         ylabel("y axis");
%         zlabel("z axis");
%         title('3D model of interior surfaces framework','FontSize',24);
%         x1=[edge_cell{i}(j,1),edge_cell{i}(j,4)];
%         y1=[edge_cell{i}(j,2),edge_cell{i}(j,5)];
%         z1=[edge_cell{i}(j,3),edge_cell{i}(j,6)];
%         plot3(x1 ,y1,z1,'b','LineWidth',1);
%         axis equal;
%         view(-114,24);
%         hold on;
%     end
% end
% axis equal;

% figure;
% for i=1:1:size(edge_points_segment,2)
% % for i=1:1:1
%     for j=1:1:size(edge_points_segment{i},2)
%         for k=1:1:size(edge_points_segment{i}{j},1)
%             xlabel("x axis");
%             ylabel("y axis");
%             zlabel("z axis");
%             title('3D model of interior surfaces framework','FontSize',24);
%             x1=[edge_points_segment{i}{j}(k,1),edge_points_segment{i}{j}(k,4)];
%             y1=[edge_points_segment{i}{j}(k,2),edge_points_segment{i}{j}(k,5)];
%             z1=[edge_points_segment{i}{j}(k,3),edge_points_segment{i}{j}(k,6)];
%             plot3(x1 ,y1,z1,'b','LineWidth',1);
%             axis equal;
%             view(-114,24);
%             hold on;
%         end
%     end
% end







