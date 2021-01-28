function [uav_path1, uav_path2, distance, time, stops] = uav_compute_path3D(xyS, routeS, home_x, home_y, x, y, wp_c, scp_table, SolL, v_op, showResults, ugv_path, uav_data, type, saveResults, fullname)

uav_path1 = [];
uav_path2 = [];
stops = 0;

if (showResults)
    %fig2 = figure;
    %plot(xyS(routeS,1),xyS(routeS,2),'black-+',home_x,home_y,'black*',x,y,'blue.');
    %hold on;
    fig3 = figure;
    plot(xyS(routeS,1),xyS(routeS,2),'black-+',home_x,home_y,'black*',x,y,'blue.');
    hold on;
end

distance = 0;
time = 0;
wp_c_aux = wp_c;
[r,c] = size(scp_table);

for i=1:c
    for k=1:length(SolL)
        if (i==SolL(k))
            subpath = [v_op(1,i);v_op(2,i)]; %Catch the vertice of the solution
            for j=1:r
                if (scp_table(j,i)==1 && wp_c_aux(1,j) ~= -1) %Waypoint j
                    subpath = [subpath [wp_c(1,j);wp_c(2,j)]];
                    wp_c_aux(1,j) = -1;
                end
            end
            
            %Shortest path to the UAV
            [rteT, dis, t, st] = search_uav_operations(subpath, uav_data); %TIME BASED SEARCHING
            stops = stops + st;
            distance = distance + dis;
            time = time + t;
            
            %UAV Path for Distance-Based Searching or Time-Based Searching
            [ru,~] = size(ugv_path);
            for z=1:ru
                if (ugv_path(z,:)==v_op(:,i)')
                    xy_uav = [subpath(1,:)',subpath(2,:)'];
                    %uav_path1 = [ uav_path1 struct('Coordinates',[xy_uav(rteP,1) xy_uav(rteP,2)])];
                    uav_path2 = [ uav_path2 struct('Coordinates',[xy_uav(rteT,1) xy_uav(rteT,2)])];
                end
            end
            
            %%% START Drawing
            if (showResults)
                %figure(fig2);
                %xy_uav = [subpath(1,:)',subpath(2,:)'];
                %plot(xy_uav(rteP,1),xy_uav(rteP,2),'blue:');
                figure(fig3);
                xy_uav = [subpath(1,:)',subpath(2,:)'];
                plot(xy_uav(rteT,1),xy_uav(rteT,2),'red:');
%               figure;
%               [~,c] = size(rte);
%               plot(1:c,rte-1,'black-+');
%               title(sprintf('UAV SubPath ID[%d] - Fc = %0.4f\n',t,minD));
%               t = t + 1;
            end
            %%% STOP Drawing
        end
    end
end
if (showResults)
    %figure(fig2);
    %title(sprintf('UAV Paths Distance-Based Searching [fuav=%1.3f]\n',fuav));
    %axis equal;
    %hold off;
    %axis equal;
    figure(fig3);
    title(sprintf('[STEP 5] - (%s) UAV Paths Time-Based [fuav=%1.3f hours]\n',type, time));
    axis equal;
    hold off;
    axis equal;
    
    if (saveResults)
        tp = char(type);
        dir = strcat(fullname,'/','uav_ts_',tp,'.fig');
        saveas(gcf,dir,'fig');
    end
end


end

