%  function [uav_paths, subtour_distance, subtour_stops] = uav_compute_3Dpath(T, scp_table, SolL, Vres, ugv_path, R)
%  This function computes the UAV's path for every subtour located on every
%  vertice. First, it selects the vertice of the solution and then the
%  target points associated to this vertice. Second, it computes the TSP
%  for this subtour.
%  Inputs:
%    T(x;y;z)    - set of target points in meters
%    scp_table   - matrix with the results of the SCP
%    SolL        - Identification in scp_table of the SCP vertices solution
%    Vres        - set of vertices solved and located at the original
%                  position in the SCP
%    ugv_path    - ordered route of the UGV
%    uav_data    - uav parameters 
%    R           - farthest distance the UAV can travel in meters
%    map_data    - map data info
%  Outputs:
%    uav_paths      - UAV routes for every subtour of the problem
%    total_distance - distance travelled by the UAV around the set of
%                     subtours
%    total_stops    - number of intermediate stops accomplished by the UAV to complete
%                     the set of subtours
function [uav_paths, total_distance, total_stops] = uav_compute_3Dpath(T, scp_table, SolL, Vres, ugv_path, uav_data, map_data)

uav_paths = [];
total_stops = 0;
total_distance = 0;
wp_c_aux = T;
[r,c] = size(scp_table);

for i=1:c
    for k=1:length(SolL)
        if (i==SolL(k))
            subpath = [Vres(1,i);Vres(2,i);Vres(3,i)]; %Subtour's Vertice
            for j=1:r
                if (scp_table(j,i)==1 && wp_c_aux(1,j) ~= -1) %Catch the target points associated to this subtour
                    subpath = [subpath [T(1,j);T(2,j);T(3,j)]];
                    wp_c_aux(1,j) = -1;
                end
            end
            %Shortest path to the UAV
            [rteT, dis, st] = search_uav_3Dpath(subpath, uav_data); %DISTANCE SEARCHING
            [uav_path,flight_z] = reconstruct_UAVpath(subpath,rteT, map_data, uav_data);
            
            total_stops = total_stops + st;
            dis = dis + 2*flight_z + 2*st*flight_z;
            total_distance = total_distance + dis;
            
            %UAV Path for Distance-Based Searching or Time-Based Searching
            [~,rc] = size(ugv_path);
            for z=1:rc
                if (ugv_path(1:2,z)==int64(Vres(1:2,i)))
                    uav_paths = [ uav_paths struct('Coordinates',uav_path)];
                end
            end
        end
    end
end
end

function [uav_path, flight_z] = reconstruct_UAVpath(subpath,rte,map_data,uav_data)

[~,c] = size(rte);
flight_z = compute_z(subpath, map_data, uav_data);
%flight_z = max(subpath(3,:)) + 10; %Flight height of the UAV 
uav_path = subpath(:,1); %Initial point
uav_path = [uav_path [subpath(1:2,1);flight_z]];
for i=1:c-1
  if (rte(i)==1 && i~=1) %Intermediate Charging stop
      uav_path = [uav_path [subpath(1:2,1);flight_z]];
      uav_path = [uav_path subpath(:,1)];
      uav_path = [uav_path [subpath(1:2,1);flight_z]];
  elseif (rte(i)>1) %Normal node
      uav_path = [uav_path [subpath(1:2,rte(i));flight_z]];
  end
end

uav_path = [uav_path [subpath(1:2,1);flight_z]]; %Final point
uav_path = [uav_path subpath(:,1)];

end

function [flight_z] = compute_z(subpath,map_data,uav_data)

% Line's Equation between two points
% (x - Xm1)/(Xm2 - Xm1) = (y - Ym1)/(Ym2 - Ym1)

syms x y z
vars = [x y];
[~,c] = size(subpath);
z_list = [];
for i=1:c-1
    eqL = ((x-subpath(1,i))/(subpath(1,i+1)-subpath(1,i)))==((y-subpath(2,i))/(subpath(2,i+1)-subpath(2,i)));
    if (subpath(1,i)>subpath(1,i+1))
        from = subpath(1,i+1);
        to = subpath(1,i);
    else
        from = subpath(1,i);
        to = subpath(1,i+1);
    end
    if (from~=to)
        for j=from:to
            y_sol = int64(solve(subs(eqL,x,j)));
            z_list = [z_list map_data.z_map(j,y_sol)];
        end
    end
end

flight_z = max(z_list) + uav_data.Z;

end

