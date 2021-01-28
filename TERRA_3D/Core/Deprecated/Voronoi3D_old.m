function [ V1 ] = Voronoi3D( T, Home, R, gmap)
%  function [ V ] = Voronoi3D( T, Home, R, map, gmap)
%  This function do the voronoi tessellations to find the minimal set of
%  vertices/charging stops covering the whole set of T.
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    Home(x;y;z) - home location of the multiple UGV-UAV system in meters
%    R    - Radius defining the maximum distance the UAV can travel in meters
%    map  - Matrix with the map points (x,y,z) in meters.
%    gmap - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    V1    - set of vertices in meters

%Algorithm variables
V1 = [];
wp_c = [];
Vx = [];
Vy = [];
Vz = [];
duplicates = 0;
vert_unleg = [];
near_wd = [];
wp_nc = T;
wp_nc_cnt = length(wp_nc(1,:));
art_vert = Home;
v_used = [];
[xmap,ymap] = size(gmap);

while ( 0 < wp_nc_cnt  )
    
    % [STEP 1] - Calculate the Voronoi's Diagram over the WPm not covered
    
    if (~isempty(Vx)) %For n iterations
        
        %Search and catch the nearest vertice to each wp_nc and add to
        %near_v
        [~,c_wp] = size(wp_nc);
        [~,c_vx] = size(Vx);
        near_v = [];
        t = 1;
        dminT = Inf;
        for i=1:c_wp
            dmin = sqrt( ((wp_nc(1,i)-Vx(1))^2) + ((wp_nc(2,i)-Vy(1))^2) + ((wp_nc(3,i)-Vz(1))^2));
            near_v(:,i) = [Vx(1);Vy(1);Vz(1)];
            for j=1:c_vx
                d = sqrt( ((wp_nc(1,i)-Vx(j))^2) + ((wp_nc(2,i)-Vy(j))^2) + ((wp_nc(3,i)-Vz(j))^2));
                if (d < dmin)
                    dmin = d;
                    near_v(:,t) = [Vx(j);Vy(j);Vz(j)];
                    t = t + 1;
                end
            end
            if (dmin < dminT)
                dminT = dmin;
            end
        end
        
        %Delete duplicates in near_v and add them to near_wd
        near_wd = delete_Duplicates(near_v);
        
        %Detect duplicates and so, infinite loop in voronoi diagram
        [~,lv] = size(near_wd);
        [~,lu] = size(v_used);
        duplicates = 0;
        for i=1:lv
            for j=1:lu
                if (near_wd(:,i)==v_used(:,j))
                    duplicates = duplicates + 1;
                end
            end
        end
        v_used = [v_used near_wd];
    end
    
    %Add these vertices to the next voronoi iteration
    wp_plus_v = [wp_nc near_wd vert_unleg];
    
    %We can't do voronoi if we have 2 points or less (one wp and its nearest vertice)
    %or Only 2 wp and 1 vértice, and the distance between the WPs and their common nearest
    %vertice, is less than the R. This situation cause an infinite loop.
    if ((length(wp_plus_v(1,:))<=2)||(duplicates > 0))
        
        if (length(wp_plus_v(1,:))==1)
            wp_plus_v(:,2) = Home;
        end
        
        %m is the intermediate point between the WPs. Each iteration,
        %this vertice is nearest to the first wp
        m_x = (wp_plus_v(1,1) + wp_plus_v(1,2)) / 2;
        m_y = (wp_plus_v(2,1) + wp_plus_v(2,2)) / 2;
        m_z = gmap(int32(m_x),int32(m_y));
        
        if (m_z~=0)
            d = sqrt( ((wp_plus_v(1,1)-m_x)^2) + ((wp_plus_v(2,1)-m_y)^2) + ((wp_plus_v(3,1)-m_z)^2));
            while (d > R)
                m_x = (m_x + wp_plus_v(1,1)) / 2;
                m_y = (m_y + wp_plus_v(2,1)) / 2;
                m_z = gmap(int32(m_x),int32(m_y));
                if (m_z~=0)
                    d = sqrt( ((wp_plus_v(1,1)-m_x)^2) + ((wp_plus_v(2,1)-m_y)^2) + ((wp_plus_v(3,1)-m_z)^2));
                else
                    %Search a near node in a green zone
                    [Vgz] = green_zones_search([int32(m_x),int32(m_y),0],wp_plus_v(:,1),R,gmap);
                end
            end
        else
            %Search a near node in a green zone
            [Vgz] = green_zones_search([int32(m_x),int32(m_y),0],wp_plus_v(:,1),R,gmap);
        end
        art_vert = [art_vert [int32(m_x);int32(m_y);gmap(int32(m_x),int32(m_y))]];
        
    else
        %2-D Voronoi Tessellation iteration
        [vx_s,vy_s] = voronoi(wp_plus_v(1,:),wp_plus_v(2,:));
        [~,cx] = size(vx_s);
        vz_s = [];
        %Catalog the type of every vertice
        vert_unleg = [];
        for i=1:cx
            if ((0<=int32(vx_s(1,i))) && (int32(vx_s(1,i))<=xmap) && (0<=int32(vy_s(1,i))) && (int32(vy_s(1,i))<=ymap))
                vz_s(1,i) = gmap(int32(vx_s(1,i)), int32(vy_s(1,i)));
                if (vz_s(1,i) == 0)
                    vz_s(1,i) = -1;
                    vert_unleg = [vert_unleg double([int32(vx_s(1,i));int32(vy_s(1,i));0])];
                end
            else
                vz_s(1,i) = -1;
            end
        end
        
        %Delete the duplicates vertices and out of boundaries vertices
        Vp = delete_Duplicates([vx_s(1,:); vy_s(1,:); vz_s]);
        if (~isempty(Vp)) 
            Vx = Vp(1,:);
            Vy = Vp(2,:);
            Vz = Vp(3,:);
        end
    end
    
    %Add artificial vertices created in the last iteration
    Vx = double([art_vert(1,:) Vx]);
    Vy = double([art_vert(2,:) Vy]);
    Vz = double([art_vert(3,:) Vz]);
    
    %Compute the new set of uncovered waypoints
    [r,c] = size(wp_nc);
    pre_list = [];
    %First, we add to pre_list the best vertices in this iteration to cover to existing target points
    for i=1:c
        dmin = R;
        best_v = -1;
        wp_v = -1;
        [~,t] = size(Vx);
        for j=1:t
            if (wp_nc(1,i)~=-1)
                d = sqrt( ((wp_nc(1,i)-Vx(j))^2) + ((wp_nc(2,i)-Vy(j))^2) + ((wp_nc(3,i)-Vz(j))^2));
                if (d < dmin && best_v ~= 1) %%Home=1 is the best option
                    dmin = d;
                    best_v = j;
                    wp_v = i;
                end
            end
        end
        if (best_v~=-1)
%             if (sum(ismember(pre_list,best_v))==0)
%                 pre_list = [pre_list best_v];
%             end

            wp_c = [wp_c wp_nc(:,wp_v)];
            wp_nc(:,wp_v) = -1;
            wp_nc_cnt = wp_nc_cnt - 1;
            V1 = [V1 [Vx(1,best_v);Vy(1,best_v);Vz(1,best_v)]];

        end
    end
    
    %Second, check if those vertices are in green zones guaranteeing the
    %covering of their target points
%    [~,c] = size(pre_list);
%     for i=1:c
%         [~,t] = size(wp_nc);
%         wp_list = [];
%         for j=1:t
%             if (wp_nc(1,j)~=-1)
%                 d = sqrt( ((wp_nc(1,j)-Vx(pre_list(i)))^2) + ((wp_nc(2,j)-Vy(pre_list(i)))^2) + ((wp_nc(3,j)-Vz(pre_list(i)))^2));
%                 if (d<R)
%                     wp_list = [wp_list wp_nc(:,j)];
%                 end
%             end
%         end
%         v_nominee = [Vx(pre_list(i));Vy(pre_list(i));Vz(pre_list(i))];
%         [Vgz] = green_zones_search(v_nominee,wp_list,R,gmap);
%         
%     % Third, update the wp_c and V1 list with the official vertices in green zones.        
%         if (Vgz ~= -1)
%             [~, f] = size(wp_list);
%             for p=1:f
%                 [~, g] = size(wp_nc);
%                 for k=1:g
%                     if (wp_list(:,p)==wp_nc(:,k))
%                         wp_nc(:,k) = -1;
%                         wp_nc_cnt = wp_nc_cnt - 1;
%                     end
%                 end
%             end
%             wp_c = [wp_c wp_list];
%             V1 =[V1 Vgz];
%         end
%     end
    
    [~,c] = size(wp_nc);
    del = 0; %To count the number of wp covered this iteration
    for i=1:c
        if (wp_nc(1,i)==-1)
            del = del + 1;
        end
    end
    
    
    %Update wp_nc Table for the next iteration
    if (del>0)
        tmp_wp_nc = wp_nc;
        wp_nc = zeros(r,c-del);
        for i=1:r
            t_y = 0;
            for j=1:c
                if (tmp_wp_nc(i,j)~=-1)
                    t_y = t_y + 1;
                    wp_nc(i,t_y) = tmp_wp_nc(i,j);
                end
            end
        end
    end
    
end

end

function [v_sol] = delete_Duplicates(vertices)
%Delete duplicates vertices
p = 1;
v_sol = [];
[~,c] = size(vertices);
for i=1:c
    v = vertices(:,i);
    in = 0;
    [~,c] = size(v_sol);
    if (v(3,1)~=-1)
        for j=1:c
            if (v==v_sol(:,j))
                in = in + 1;
            end
        end
        if in==0
            v_sol(:,p) = v;
            p = p + 1;
        end
    end
end
end



