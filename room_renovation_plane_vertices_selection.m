function [room_renovation_facet,room_renovation_vertices,room_renovation_norm_vector]=room_renovation_plane_vertices_selection(room_facet,room_vertices,room_norm_vector)
m=1;
for i=1:1:size(room_norm_vector,2)
    for j=1:1:size(room_norm_vector{i},1)
        vertices_flag{i}(j,1)=0;
    end
end
% the horizontal plane
% for i=1:1:size(room_norm_vector,2)
%     for j=1:1:size(room_norm_vector{i},1)
%         if abs(room_norm_vector{i}(j,1))<=0.01 && abs(room_norm_vector{i}(j,2))<=0.01 && abs(room_norm_vector{i}(j,3))<=1.01
%             vertices_flag{i}(j,1)=1;
%         end
%     end
% end
% % the other plane 
% for i=1:1:size(room_norm_vector,2)
%     for j=1:1:size(room_norm_vector{i},1)
%         if abs(room_vertices{i}(3*j-2,1))<=0.01 && abs(room_vertices{i}(3*j-1,1))<=0.01 && abs(room_vertices{i}(3*j,1))<=0.01
%             vertices_flag{i}(j,1)=1;
%         end
%         if abs(room_vertices{i}(3*j-2,2)-1500)<=0.01 && abs(room_vertices{i}(3*j-1,2)-1500)<=0.01 && abs(room_vertices{i}(3*j,2)-1500)<=0.01
%             vertices_flag{i}(j,1)=1;
%         end
%     end
% end

for i=1:1:size(room_norm_vector,2)
    k=1;
    for j=1:1:size(room_norm_vector{i},1)
        if vertices_flag{i}(j,1)==0
            room_renovation_facet{i}(k,1:3)=[3*k-2,3*k-1,3*k];
            room_renovation_norm_vector{i}(k,1:3)=room_norm_vector{i}(j,1:3);
            room_renovation_vertices{i}(3*k-2:3*k,1:3)=room_vertices{i}(3*j-2:3*j,1:3);
            k=k+1;
        end
    end
end


end
