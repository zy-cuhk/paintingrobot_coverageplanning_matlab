function [house_facet02,house_vertices02,house_norm_vector02]=house_process(house_facet,house_vertices,house_norm_vector)


%% the first part: house process
house_facet_num=size(house_facet,1);
house_facet_new=zeros(house_facet_num,3);
house_vertices_new=zeros(3*house_facet_num,3);
house_norm_vector_new=zeros(house_facet_num,3);
for i=1:1:house_facet_num
    house_facet_new(i,:)=house_facet(i,:);
    
    house_vertices_new(3*i-2,1)=house_vertices(3*i-2,1);
    house_vertices_new(3*i-2,2)=house_vertices(3*i-2,3);
    house_vertices_new(3*i-2,3)=house_vertices(3*i-2,2);
    
    house_vertices_new(3*i-1,1)=house_vertices(3*i-1,1);
    house_vertices_new(3*i-1,2)=house_vertices(3*i-1,3);
    house_vertices_new(3*i-1,3)=house_vertices(3*i-1,2);
    
    house_vertices_new(3*i,1)=house_vertices(3*i,1);
    house_vertices_new(3*i,2)=house_vertices(3*i,3);
    house_vertices_new(3*i,3)=house_vertices(3*i,2);
    
    house_norm_vector_new(i,1)=house_norm_vector(i,1);
    house_norm_vector_new(i,2)=-house_norm_vector(i,3);
    house_norm_vector_new(i,3)=house_norm_vector(i,2);
end

m=1;
for i=1:1:house_facet_num
    if abs(house_norm_vector_new(i,3))<0.01 
        house_norm_vector_new01(m,1:3)=house_norm_vector_new(i,1:3);
        house_vertices_new01(3*m-2:3*m,1:3)=house_vertices_new(3*i-2:3*i,1:3);
        house_facet_new01(m,1:3)=[3*m-2:3*m];
        m=m+1;
    end
end


house_facet02=house_facet_new01;
house_facet_num=size(house_facet_new01,1);
house_vertices02=zeros(3*house_facet_num,3);
house_vertices02(:,1)=house_vertices_new01(:,1)-min(house_vertices_new01(:,1));
house_vertices02(:,2)=house_vertices_new01(:,2)-min(house_vertices_new01(:,2));
house_vertices02(:,3)=house_vertices_new01(:,3)-min(house_vertices_new01(:,3));
house_norm_vector02=house_norm_vector_new01;




end