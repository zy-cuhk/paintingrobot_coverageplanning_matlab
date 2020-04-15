clc,clear all,close all;
tic;

%% generating room planes of interior environment
[house_facet,house_vertices,house_norm_vector]=house_stl_reading("202003experiment.STL");
% house_stl_matplot(house_facet,house_vertices,house_norm_vector);

[room_facet,room_vertices,room_norm_vector]=room_walls_obtaining(house_facet,house_vertices,house_norm_vector);
[room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_edge_cell]=room_planes_generation(room_facet,room_vertices,room_norm_vector);
% room_wallplanes_visualization(room_plane_edge_cell);

%% generating renovation planes inside the interior environment
panning_distance1=400;
[ground_edge_cells,renovation_planes_edge_cell,renovation_planes_norm_vector,renovation_plane_triangle_edges]=renovation_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_edge_cell,panning_distance1);
panning_distance2=1000;
[renovation_manipulatorbase_planes]=renovation_manipulatorbase_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_edge_centroid,room_plane_triangle_edge_cell,panning_distance2);

%% generating waypoints on the room plane
waypoints_interval=50;path_interval=100;
[room_plane_edge_point_cell1,room_plane_edge_cell1,room_plane_norm_vector1,room_triangle_cell1,room_triangle_edge_cell1,triangle_norm_vector_cell1]=room_planes_generation1(room_facet,room_vertices,room_norm_vector);
[room_effective_waypoints,room_effective_waypaths,room_plane_boundary]=room_renovation_planes_waypoints_generation(room_plane_edge_cell1,room_plane_norm_vector1,room_triangle_cell1,room_triangle_edge_cell1,triangle_norm_vector_cell1,room_vertices,waypoints_interval,path_interval);

%% selecting the required planes 
[renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_planes_norm_vector,renovation_effective_waypoints,renovation_effective_waypaths]=planes_selection(renovation_planes_edge_cell,renovation_manipulatorbase_planes,renovation_planes_norm_vector,room_effective_waypoints,room_effective_waypaths,panning_distance1);

%% renovation waypaths and waypoints are clustered together again.
% for i=1:1:size(renovation_cells_waypaths,2)
%     for j=1:1:size(renovation_cells_waypaths{i},2)
%         for k=1:1:size(renovation_cells_waypaths{i}{j},2)
%             a=renovation_cells_waypaths{i}{j}{k};
%             b=path_clustering(a);
%             renovation_cells_clustering_waypaths{i}{j}{k}=b;
%         end
%     end
% end

ymin=0; ymax=2.7; interval1=0.1;
zmin=0; zmax=3; interval2=0.2;m=1;
% ymin:0, ymax:2700,x:1100,z=0,roll=0,pitch=0,yaw=0
for i=1:1:(ymax-ymin)/interval1
    for j=1:1:(zmax-zmin)/interval2
        renovation_manipulatorbase_points(m,1)=1.10;
        renovation_manipulatorbase_points(m,2)=i*interval1;
        renovation_manipulatorbase_points(m,3)=j*interval2;
        renovation_manipulatorbase_points(m,4)=0;
        renovation_manipulatorbase_points(m,5)=0;
        renovation_manipulatorbase_points(m,6)=0;
        m=m+1;
    end
end

room_renovation_plane_waypoints_visualization(renovation_manipulatorbase_points,renovation_effective_waypoints,room_plane_edge_cell,renovation_planes_edge_cell,renovation_effective_waypaths);


save("data.mat",'renovation_effective_waypoints','renovation_effective_waypaths','renovation_manipulatorbase_points');


toc;







