%  function [ V4, vres] = Gravitational_Optimization_3D( T, Home, V2, Gp, SolL, scp_table, R, gmap)
%  This function computes a distance reduction for every vertice in the UGV's
%  path.
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    Home(x;y;z) - home location of the multiple UGV-UAV system in meters
%    V2(x;y;z)   - set of vertices solved in greedy SCP in meters
%    Gp          - set of considered gravity points
%    SolL        - contains the labels corresponding to the vertices
%                  solutions in V2
%    scp_table   - table identifying the scp solution for each couple
%                  vertice/target point
%    R           - Radius defining the maximum distance the UAV can travel in meters
%    map_data    - Map data info
%    cfg_params  - testing configuration parameters
%  Outputs:
%    V3          - set of vertices solved in this stage in meters
%    Vres        - set of vertices solved and located at the original
%                  position in the SCP
%    figO        - figures solution


function [V3, Vres, figO] = Gravitational_Optimization_3D( T, Home, V2, Gp, SolL, scp_table, R, map_data, cfg_params)

T_tmp = T;
[r,c] = size(scp_table);
Vres = zeros(3,c);
V3 = [];

for i=1:c
    for k=1:length(SolL)
        a = [];
        b = [];
        c = [];
        if (i==SolL(k) && ~isequal(V2(:,i),Home))
            Xm = [V2(1,i) Gp(1,1)];
            Ym = [V2(2,i) Gp(2,1)];
            Zm = [V2(3,i) Gp(3,1)];
            if (~isequal(V2(:,1),Gp(:,1)))
                for j=1:r
                    if (scp_table(j,i)==1 && T_tmp(1,j) ~= -1) %Waypoint j
                        a = [a T(1,j)];
                        b = [b T(2,j)];
                        c = [c T(3,j)];
                        T_tmp(1,j) = -1;
                    end
                end
                if (map_data.map_type == 0)
                    [Vopt] = foptimus_3D(a, b, c, Xm, Ym, Zm, R, map_data);
                else
                    [Vopt] = foptimus_2D(a, b, Xm, Ym, R, map_data);
                end
                if (isempty(Vopt))
                    Vopt = V2(:,i);
                end
            else
                Vopt = V2(:,i);
            end
            V3 = [V3 Vopt];
            Vres(:,i) = Vopt;
        elseif (i==SolL(k) && isequal(V2(:,i),Home))
            Vres(:,i) = V2(:,i);
            %V3 = [Home V3];
        end
    end
end

%Add Home to the vertices solution
V3 = [Home V3];

%Print and Save Results
figO = [];
if (cfg_params.printResults)
    vis = 'on';
else
    vis = 'off';
end

fig = figure('visible',vis);
h = axes;
fig.Name = 'GravOpt_Solution';
hold on
contourf(map_data.z_map',75);
mcolor = jet;
%mcolor(1,:) = [1,1,1];
colormap(mcolor);
colorbar('eastoutside')
plot(T(1,:),T(2,:),'m.','MarkerSize',20);
if (~isempty(V3))
    plot(V3(1,:),V3(2,:),'red+','MarkerSize',10,'LineWidth',3);
end
if (~isempty(V2))
    %plot(V2(1,:),V2(2,:),'black+','MarkerSize',10,'LineWidth',3);
end
plot(Home(1,1),Home(2,1),'black*','MarkerSize',15,'LineWidth',2);
[~,c]= size(V3);
theta = linspace(0,2*pi);
for i=1:c
    c_x(i,:) = R*cos(theta) + V3(1,i);
    c_y(i,:) = R*sin(theta) + V3(2,i);
    plot(c_x(i,:),c_y(i,:),'r:','LineWidth',2);
end

if (map_data.map_type == 1)
    %set(h, 'Ydir', 'reverse')
    xlabel('X-Axis in Map')
    ylabel('Y-Axis in Map')
    zlabel('Z-Axis in Map')
else
    xlabel('X-Axis in Map')
    ylabel('Y-Axis in Map')
    zlabel('Z-Axis in Map')
end

title('Gravitational Optimization Solution');
hold off
axis equal

if (cfg_params.saveResults)
    %figO = [figO fig];
end

end

