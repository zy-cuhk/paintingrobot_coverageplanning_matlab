function [room_facet,room_vertices,room_norm_vector]=room_walls_obtaining(house_facet,house_vertices,house_norm_vector)
%% the first part: house process
house_facet_num=size(house_facet,1);
house_facet_new=zeros(house_facet_num,3);
house_vertices_new=zeros(3*house_facet_num,3);
house_norm_vector_new=zeros(house_facet_num,3);
for i=1:1:house_facet_num
    house_facet_new(i,:)=house_facet(i,:);
    
    house_vertices_new(3*i-2,1)=-house_vertices(3*i-2,1);
    house_vertices_new(3*i-2,2)=house_vertices(3*i-2,3);
    house_vertices_new(3*i-2,3)=house_vertices(3*i-2,2);
    
    house_vertices_new(3*i-1,1)=-house_vertices(3*i-1,1);
    house_vertices_new(3*i-1,2)=house_vertices(3*i-1,3);
    house_vertices_new(3*i-1,3)=house_vertices(3*i-1,2);
    
    house_vertices_new(3*i,1)=-house_vertices(3*i,1);
    house_vertices_new(3*i,2)=house_vertices(3*i,3);
    house_vertices_new(3*i,3)=house_vertices(3*i,2);
    
    house_norm_vector_new(i,1)=house_norm_vector(i,1);
    house_norm_vector_new(i,2)=-house_norm_vector(i,3);
    house_norm_vector_new(i,3)=house_norm_vector(i,2);
end
house_facet=house_facet_new;
house_vertices=zeros(3*house_facet_num,3);
house_vertices(:,1)=house_vertices_new(:,1)-min(house_vertices_new(:,1));
house_vertices(:,2)=house_vertices_new(:,2)-min(house_vertices_new(:,2));
house_vertices(:,3)=house_vertices_new(:,3)-min(house_vertices_new(:,3));
house_norm_vector=house_norm_vector_new;


%% the second part: room segment 
% xyz_minmax is written as: [xmin, xmax, ymin, ymax, zmin, zmax];
xyz_minmax=[ 5, 3650, 5, 4800, -5, 3000]; % these parameters are for 0016.stl

for i=1:1:size(xyz_minmax,1)
    j=1;
    triangle_mesh_num=size(house_facet,1);
    for m=1:1:triangle_mesh_num
        if all(house_vertices(3*m-2:3*m,1)>xyz_minmax(i,1)) && all(house_vertices(3*m-2:3*m,1)<xyz_minmax(i,2))
            if all(house_vertices(3*m-2:3*m,2)>xyz_minmax(i,3)) && all(house_vertices(3*m-2:3*m,2)<xyz_minmax(i,4))
                if all(house_vertices(3*m-2:3*m,3)>xyz_minmax(i,5)) && all(house_vertices(3*m-2:3*m,3)<xyz_minmax(i,6))
                    if abs(abs(house_norm_vector(m,3))-1)>0.1
                        room_facet{i}(j,1:3)=[3*j-2, 3*j-1, 3*j];
                        room_vertices{i}(3*j-2:3*j,1:3)=house_vertices(3*m-2:3*m,1:3);
                        room_norm_vector{i}(j,1:3)=house_norm_vector(m,1:3);
                        j=j+1;
                    end
                end
            end
        end
    end
end
%% the third part: room process
room_facet_1=room_facet;
room_vertices_1=room_vertices;
room_norm_vector_1=room_norm_vector;
for i=1:1:size(room_facet_1,2)
    room_facet{i}=room_facet_1{i};
    room_vertices{i}(:,1)=room_vertices_1{i}(:,1)-min(room_vertices_1{i}(:,1));
    room_vertices{i}(:,2)=room_vertices_1{i}(:,2)-min(room_vertices_1{i}(:,2));
    room_vertices{i}(:,3)=room_vertices_1{i}(:,3)-min(room_vertices_1{i}(:,3));
    room_norm_vector{i}=room_norm_vector_1{i};
end

end