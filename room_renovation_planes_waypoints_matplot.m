function room_renovation_planes_waypoints_matplot(room_effective_waypoints,room_plane_boundary)
figure;

for i=1:1:size(room_effective_waypoints,2)
    x=room_effective_waypoints{i}(:,1);
    y=room_effective_waypoints{i}(:,2);
    z=room_effective_waypoints{i}(:,3);
    scatter3(x,y,z);
    hold on;
end

for i=1:1:size(room_plane_boundary,2)
    for j=1:1:size(room_plane_boundary{i},2)
        for k=1:1:size(room_plane_boundary{i}{j},1)
            x1=[room_plane_boundary{i}{j}(k,1),room_plane_boundary{i}{j}(k,4)];
            y1=[room_plane_boundary{i}{j}(k,2),room_plane_boundary{i}{j}(k,5)];
            z1=[room_plane_boundary{i}{j}(k,3),room_plane_boundary{i}{j}(k,6)];
            plot3(x1,y1,z1);
            hold on;
        end
    end
end
axis equal;

end


