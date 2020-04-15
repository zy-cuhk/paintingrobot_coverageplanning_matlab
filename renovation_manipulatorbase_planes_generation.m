function [renovation_manipulatorbase_planes]=renovation_manipulatorbase_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_edge_cell,panning_distance)

%% the below is to generate renovation_planes_edge_cell and renovation_planes_norm_vector
% input: room_plane_edge_cell and room_plane_norm_vector
% input: panning_distance(the panning distance between room target plane and its panning renovation plane)
% output:renovation_planes_edge_cell and renovation_planes_norm_vector
renovation_planes_edge_cell={};
%% firstly room planes are divided into two categories, including horizontal planes and vertical planes
m=1;n=1;
for i=1:1:size(room_plane_norm_vector,2)
    
    flag=0;
    n1=room_plane_norm_vector{i}(1,1);
    n2=room_plane_norm_vector{i}(1,2);
    n3=room_plane_norm_vector{i}(1,3);
    if abs(n1)<=0.01 && abs(n2)<=0.01 && abs(n3)-1<=0.01
        flag=1;
    end
    if flag==0
        room_vplane_norm_vector{n}=room_plane_norm_vector{i};
        room_vplane_edge_cell{n}=room_plane_edge_cell{i};
        room_vplane_edge_centroid{n}=room_plane_edge_centroid{i};
        room_vplane_triangle_edges{n}=room_plane_triangle_edge_cell{i};
        n=n+1;
    end
end

%% secondly horizontal planes are divided into two kinds: ground plane and non-ground planes
ground_edge_cells=[];
for i=1:1:size(room_vplane_edge_cell,2)
    n=1; 
    for j=1:1:size(room_vplane_edge_cell{i},1)
        xyz_vector(n,1)=room_vplane_edge_cell{i}(j,1);
        xyz_vector(n,2)=room_vplane_edge_cell{i}(j,2);
        xyz_vector(n,3)=0;
        n=n+1;
    end
    xyz_vector=unique(xyz_vector,'rows');
    xyz_vector=sortrows(xyz_vector);
    ground_edge_cells(i,1:3)=xyz_vector(1,1:3);
    ground_edge_cells(i,4:6)=xyz_vector(end,1:3);
end

%% fourthly the processing of planar parameters of room vertical planes
hplane_line=ones(size(ground_edge_cells,1),4);
for i=1:1:size(ground_edge_cells,1)
    p1=ground_edge_cells(i,1:3);
    p2=ground_edge_cells(i,4:6);
    centroid=[1000,4000,0];
    
    a=p2(2)-p1(2);
    b=p1(1)-p2(1);
    c1=p2(1)*p1(2)-p1(1)*p2(2);
    c2=-(a*centroid(1)+b*centroid(2));
    % c2=-(hplane_line(i,1)*centroid(1)+hplane_line(i,2)*centroid(2));
    hplane_line(i,1)=a/sqrt(a^2+b^2);
    hplane_line(i,2)=b/sqrt(a^2+b^2);
    hplane_line(i,3)=c1/sqrt(a^2+b^2);
    hplane_line(i,4)=c2/sqrt(a^2+b^2);
end
vplane_line=ones(size(room_vplane_edge_cell,2),4);
for i=1:1:size(room_vplane_edge_cell,2)
    p1=room_vplane_edge_cell{i}(1,1:3);
    p2=room_vplane_edge_centroid{i}(1,1:3);
    a=p2(2)-p1(2);
    b=p1(1)-p2(1);
    c=p2(1)*p1(2)-p1(1)*p2(2);
    vplane_line(i,1)=a/sqrt(a^2+b^2);
    vplane_line(i,2)=b/sqrt(a^2+b^2);
    vplane_line(i,3)=c/sqrt(a^2+b^2);
end
for i=1:1:size(hplane_line,1)
    line1=hplane_line(i,1:3);
    for j=1:1:size(vplane_line,1)
        line2=vplane_line(j,1:3);
        if abs(line1(1)-line2(1))<=0.01 && abs(line1(2)-line2(2))<=0.01 && abs(line1(3)-line2(3))<=0.01
            vplane_line(j,4)=hplane_line(i,4);
        end
        if abs(line1(1)+line2(1))<=0.01 && abs(line1(2)+line2(2))<=0.01 && abs(line1(3)+line2(3))<=0.01
            vplane_line(j,4)=hplane_line(i,4);
        end
    end
end
%% fifthly, the processing of planar points of vertical planes
% input:room_vplane_edge_cell,room_vplane_norm_vector,vplane_line,panning_distance
% output:renovation_planes_edge_cell,renovation_planes_norm_vector

for i=1:1:size(room_vplane_edge_cell,2)
    a=vplane_line(i,1);
    b=vplane_line(i,2);
    c=0;
    d1=vplane_line(i,3);
    d2=vplane_line(i,4);
    for j=1:1:size(room_vplane_edge_cell{i},1)
        for k=1:1:2
            room_vplane_edge_cell{i}(j,3*k-2)=room_vplane_edge_cell{i}(j,3*k-2)+sign(d1-d2)*a/sqrt(a^2+b^2+c^2)*panning_distance;
            room_vplane_edge_cell{i}(j,3*k-1)=room_vplane_edge_cell{i}(j,3*k-1)+sign(d1-d2)*b/sqrt(a^2+b^2+c^2)*panning_distance;
            room_vplane_edge_cell{i}(j,3*k)=room_vplane_edge_cell{i}(j,3*k)+sign(d1-d2)*c/sqrt(a^2+b^2+c^2)*panning_distance;
        end
    end
    for j=1:1:size(room_vplane_triangle_edges{i},1)
        for k=1:1:2
            room_vplane_triangle_edges{i}(j,3*k-2)=room_vplane_triangle_edges{i}(j,3*k-2)+sign(d1-d2)*a/sqrt(a^2+b^2+c^2)*panning_distance;
            room_vplane_triangle_edges{i}(j,3*k-1)=room_vplane_triangle_edges{i}(j,3*k-1)+sign(d1-d2)*b/sqrt(a^2+b^2+c^2)*panning_distance;
            room_vplane_triangle_edges{i}(j,3*k)=room_vplane_triangle_edges{i}(j,3*k)+sign(d1-d2)*c/sqrt(a^2+b^2+c^2)*panning_distance;
        end
    end
end
%% sixthly, adding the above result into the renovation_planes
plane_num_now=size(renovation_planes_edge_cell,2);
for i=1:1:size(room_vplane_edge_cell,2)
    renovation_planes_edge_cell{i+plane_num_now}=room_vplane_edge_cell{i};
    renovation_planes_norm_vector{i+plane_num_now}=room_vplane_norm_vector{i};
    renovation_plane_triangle_edges{i+plane_num_now}=room_vplane_triangle_edges{i};
end
m=1;
for i=1:1:size(renovation_planes_norm_vector,2)
    a=renovation_planes_norm_vector{i}(1,1);
    b=renovation_planes_norm_vector{i}(1,2);
    c=renovation_planes_norm_vector{i}(1,3);
    if abs(c)<=0.01
        new_renovation_planes_edge_cell{m}=renovation_planes_edge_cell{i};
        new_renovation_planes_norm_vector{m}=renovation_planes_norm_vector{i};
        new_renovation_plane_triangle_edges{m}=renovation_plane_triangle_edges{i};
        m=m+1;
    end
end
%% obtain the lines function of renovation mobile platform positions
for i=1:1:size(new_renovation_planes_norm_vector,2)
    a=new_renovation_planes_norm_vector{i}(1,1);
    b=new_renovation_planes_norm_vector{i}(1,2);
    c=new_renovation_planes_norm_vector{i}(1,3);
    x0=new_renovation_planes_edge_cell{i}(1,1);
    y0=new_renovation_planes_edge_cell{i}(1,2);
    z0=new_renovation_planes_edge_cell{i}(1,3);
    d=-(a*x0+b*y0+c*z0);
    renovation_manipulatorbase_planes{i}(1,1)=a;
    renovation_manipulatorbase_planes{i}(1,2)=b;
    renovation_manipulatorbase_planes{i}(1,3)=d;
end
%% obtain


end

