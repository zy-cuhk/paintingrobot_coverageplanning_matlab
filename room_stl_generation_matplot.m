function room_stl_generation_matplot(room_facet,room_vertices,room_norm_vector)
figure;
xlabel("x axis");
ylabel("y axis");
zlabel("z axis");
title('3D model of one house','FontSize',24);

for i=1:1:size(room_norm_vector,2)
    m=1;
    for j=1:1:size(room_norm_vector{i},1)
        nz=room_norm_vector{i}(j,3);
        if abs(nz)<0.01
            room_norm_vector01{i}(m,1:3)=room_norm_vector{i}(j,1:3);
            room_vertices01{i}(3*m-2:3*m,1:3)=room_vertices{i}(3*j-2:3*j,1:3);
            room_facet01{i}(m,1:3)=[3*m-2,3*m-1,3*m];
            m=m+1;
        end
    end
end


for i=1:1:size(room_vertices01,2)
%     nz=room_norm_vector{i}(1,3);
%     if abs(nz)<0.01
        vertices=room_vertices01{i};
        facet=room_facet01{i};
        patch('Faces',facet,'Vertices',vertices,'FaceColor',[0.8 0.8 1.0], ...
            'EdgeColor','none',...
            'FaceLighting','gouraud',...
            'AmbientStrength', 0.15);
%     end
end
camlight('headlight');
material('dull');
view(-150,20);
axis equal;
hold off;


% m=1;
% for i=1:1:size(room_norm_vector01,2)
%     for j=1:1:size(room_norm_vector01{i},1)
%         room_norm_vector_new(m,1:3)=room_norm_vector01{i}(j,1:3);
%         room_vertices_new(3*m-2:3*m,1:3)=room_vertices01{i}(3*j-2:3*j,1:3)*0.001;
%         room_facet_new(m,1:3)=[3*m-2,3*m-1,3*m];
%         m=m+1;
%     end
% end

end


