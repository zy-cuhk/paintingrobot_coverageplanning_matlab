function room_renovation_pathcells_visualization(room_renovation_pathcell_points,room_renovation_pathcell_midpoints_flag,renovation_planes_edge_cell)

%% plot renovation paths on renovation planes
figure;
for i=1:1:size(room_renovation_pathcell_points,2)
% for i=5:1:5
    for m=1:1:size(room_renovation_pathcell_points{i},2)
        for n=1:1:size(room_renovation_pathcell_points{i}{m},2)
            for k=1:1:size(room_renovation_pathcell_points{i}{m}{n},1)-1
                if room_renovation_pathcell_midpoints_flag{i}{m}{n}(1,k)==1
                    x1=[room_renovation_pathcell_points{i}{m}{n}(k,1),room_renovation_pathcell_points{i}{m}{n}(k+1,1)];
                    y1=[room_renovation_pathcell_points{i}{m}{n}(k,2),room_renovation_pathcell_points{i}{m}{n}(k+1,2)];
                    z1=[room_renovation_pathcell_points{i}{m}{n}(k,3),room_renovation_pathcell_points{i}{m}{n}(k+1,3)];
%                     plot3(x1,y1,z1,'k')
%                     hold on;
                    if mod(m,2)==0
                        plot3(x1,y1,z1,'r');
                        hold on;
                    else
                        plot3(x1,y1,z1,'b');
                        hold on;
                    end
                end
            end
        end
    end
end
hold on;
for i=1:1:size(renovation_planes_edge_cell,2)
    for j=1:1:size(renovation_planes_edge_cell{i},1)
        xlabel("x axis");
        ylabel("y axis");
        zlabel("z axis");
        title('3D model of interior surfaces framework','FontSize',24);
        x1=[renovation_planes_edge_cell{i}(j,1),renovation_planes_edge_cell{i}(j,4)];
        y1=[renovation_planes_edge_cell{i}(j,2),renovation_planes_edge_cell{i}(j,5)];
        z1=[renovation_planes_edge_cell{i}(j,3),renovation_planes_edge_cell{i}(j,6)];
        plot3(x1 ,y1,z1,'k','LineWidth',1);
        axis equal;
        view(-114,24);
        hold on;
    end
end
% hold on;
% for i=1:1:size(room_plane_edge_cell,2)
% % close all;
% % for i=a:1:a
%     for j=1:1:size(room_plane_edge_cell{i},1)
%         xlabel("x axis");
%         ylabel("y axis");
%         zlabel("z axis");
%         title('3D model of interior surfaces framework','FontSize',24);
%         x1=[room_plane_edge_cell{i}(j,1),room_plane_edge_cell{i}(j,4)];
%         y1=[room_plane_edge_cell{i}(j,2),room_plane_edge_cell{i}(j,5)];
%         z1=[room_plane_edge_cell{i}(j,3),room_plane_edge_cell{i}(j,6)];
%         plot3(x1 ,y1,z1,'b','LineWidth',1);
%         axis equal;
%         view(-114,24);
%         hold on;
%     end
% end
axis equal;


end



