% This function solve an equations system among the WPs Sphere's
% Equations and the line between the points Gp and Vi

% Also, it checks if the new optimal vertice is inside the green zone of
% the map. If not, it computes the nearest vertice inside the green zone.

% Sphere's Equation
% (x-a)^2 - (y-b)^2 -(z-c)^2 = R^2

% Line's Equation between two points
% (x - Xm1)/(Xm2 - Xm1) = (y - Ym1)/(Ym2 - Ym1)

% INPUTS
% a        - vector with the a's center of the WPs spheres C(a,b,c)
% b        - vector with the b's center of the WPs spheres C(a,b,c)
% c        - vector with the c's center of the WPs spheres C(a,b,c)
% Xm       - vector with the X coordinates of the points of the vertices
% Ym       - vector with the Y coordinates of the points of the vertices
% Zm       - vector with the Z coordinates of the points of the vertices
% R        - farthest distance the UAV can travel
% map_data - Info data map

% OUTPUT
% Vopt - the optimal vertice of this set(vertice,WP1,WP2, ..., WPN)

% * This function is applied in 3D Maps, where there is an altitudes map.
function [Vopt] = foptimus_3D(a, b, c, Xm, Ym, Zm, R, map_data)

 %Output
 Vopt = [];
 
 %Variables of the system of equations
 syms x y z
 vars = [x y z];
 
 %Build the Line's equation between the vertice and the gravity point
 eqL = ((x-Xm(1))/(Xm(2)-Xm(1)))==((y-Ym(1))/(Ym(2)-Ym(1)));
 
 jp_sol = [];
 %Compute junction points of the solution
 for i=1:length(a)
     eqC = ((((x-a(i))^2) + ((y-b(i))^2) + ((z-c(i))^2)) == R^2);
     eqns = [eqC, eqL];
     [solx, soly, ~] = solve(eqns,vars);
     for j=1:length(solx)
         if (insideMapLimits(abs(solx(j)),abs(soly(j)),map_data)) 
             jp_sol = [jp_sol ; [double(floor(abs(solx(j)))) double(floor(abs(soly(j)))) map_data.z_map(double(floor(abs(solx(j)))),double(floor(abs(soly(j)))))]];
         end
     end
 end
 
 jp_cover = [];
 [r,~] = size(jp_sol);
 %For each solution, get the ones who cover all the waypoints
 for j=1:r
     wp_in = 0;
     for k=1:length(a)
         d = sqrt( ((jp_sol(j,1)-a(k))^2) + ((jp_sol(j,2)-b(k))^2) + ((jp_sol(j,3)-c(k))^2) );
         if (round(d) <= R)
             wp_in = wp_in + 1;
         end
     end
     if (wp_in == length(a))
         jp = struct('id',j,'x',jp_sol(j,1),'y',jp_sol(j,2),'z',jp_sol(j,3),'d',sqrt(((jp_sol(j,1)-Xm(2))^2) + ((jp_sol(j,2)-Ym(2))^2) + ((jp_sol(j,3)-Zm(2))^2) ));
         jp_cover = [jp_cover jp];
     end
 end

%Sort the near junction points (or related) inside the green zones.
Vjp = [];   %List of optimized junction points
Gp = [Xm(2), Ym(2), Zm(2)];
vertex = [Xm(1), Ym(1), Zm(1)];
while (~isempty(jp_cover))
    
    %Sort jp_array by distance 'd'
    jp_cover = sortStruct(jp_cover,5);
    
    %Get & Delete first jpoint
    current_jp = jp_cover(1);
    jp_cover(1).id = Inf;
    jp_cover = deletePoint(jp_cover);
    
    if (inGreenZone([current_jp.x;current_jp.y],map_data))
        Vjp = struct('id',current_jp.id,'x',current_jp.x,'y',current_jp.y,'z',current_jp.z,'d',0);
        break;
    else
        %Line between the junction point [minX;minY] and the vertice [Xm(1);Ym(1)]
        eqL = ((x-current_jp.x)/(Xm(1)-current_jp.x))==((y-current_jp.y)/(Ym(1)-current_jp.y));
        eqM = ((Ym(1)-current_jp.y)/(Xm(1)-current_jp.x));
        
        %Search in X-axis: current_jp.x,Xm(1)
        Vjp = [Vjp axiSearch(eqL,eqM,x,Gp,current_jp.x,Xm(1),map_data,current_jp,vertex,length(jp_cover))];
        
        %Search in Y-axis: current_jp.y,Ym(1)
        Vjp = [Vjp axiSearch(eqL,eqM,y,Gp,current_jp.y,Ym(1),map_data,current_jp,vertex,length(jp_cover))];
    end
end

if (~isempty(Vjp))
    %Select the nearest
    Vjp = sortStruct(Vjp,5);
    Vopt = [Vjp(1).x;Vjp(1).y;Vjp(1).z]; %Get first
end

end
function v_opt = axiSearch(eqL,eqM,var,Gp,jp_coordinate,v_coordinate,map_data,jp,vertex,id)
    v_opt = [];
    
    %Distance vertex - gp
    d1 = sqrt( ((Gp(1,1)-vertex(1,1))^2) + ((Gp(1,2)-vertex(1,2))^2) + ((Gp(1,3)-vertex(1,3))^2));
    %Distance jp - gp
    d2 = sqrt( ((jp.x-vertex(1,1))^2) + ((jp.y-vertex(1,2))^2) + ((jp.z-vertex(1,3))^2));
    
    %Select nearest vertex to the Gp
    if (d1>d2)
        from = jp_coordinate;
        to = v_coordinate;
    elseif (d1<=d2)
        from = v_coordinate;
        to = jp_coordinate;
    end
    
    %Select Order
    order = 1;
    if (from>to)
        order = -1;
    end

    %One-by-one meters
    for i=from:order:to
        if (i~=jp_coordinate)
            value = double(floor(solve(subs(eqL,var,i))));
            if (var=='x')
                if (eqM == 0)
                    value = jp.y;
                end
                if (inGreenZone([i;value],map_data))
                    v_opt = struct('id',id,'x',double(i),'y',value,'z',double(map_data.z_map(i,value)),'d',sqrt( ((double(i)-Gp(1,1))^2) + ((value-Gp(1,2))^2) + ((double(map_data.z_map(i,value))-Gp(1,3))^2) ));
                    break;
                end
            elseif (var=='y')
                if (eqM == -Inf || eqM == +Inf)
                    value = jp.x;
                end
                if (inGreenZone([value;i],map_data))
                    v_opt = struct('id',id,'x',value,'y',double(i),'z',double(map_data.z_map(value,i)),'d',sqrt( ((value-Gp(1,1))^2) + ((double(i)-Gp(1,2))^2) + ((double(map_data.z_map(value,i))-Gp(1,3))^2) ));
                    break;
                end
            end
        end
    end
end

function [list] = deletePoint(list)

L = [];
[~,c] = size(list);

for i=1:c 
    if (list(i).id ~= Inf)
        L = [L list(i)];
    end
end

list = L;

end

function [list] = sortStruct(l,idx)
    list = [];
    if (~isempty(l))
        %This function sorts a list of structs by the 'idx' field
        cells = struct2cell(l); %converts struct to cell matrix
        sortvals = cells(idx,1,:); % gets the values of just the first field
        mat = cell2mat(sortvals); % converts values to a matrix
        mat = squeeze(mat); %removes the empty dimensions for a single vector
        [~,ix] = sort(mat); %sorts the vector of values
        list = l(ix); %rearranges the original array
    end
end

function output = inGreenZone(v,map_data)
    output = 0;
    if (insideMapLimits(v(1,1),v(2,1),map_data))
        output = map_data.gz_map(v(1,1),v(2,1)) ~= 0;
    end
end

function [bool] = insideMapLimits(x,y,map_data)
    [xmap,ymap] = size(map_data.z_map);
    bool = 0;
    if (0<int64(x) && int64(x)<=xmap && 0<int64(y) && int64(y)<=ymap)
        z = map_data.z_map(int64(x),int64(y));
        if (int64(z) ~= int64(map_data.map_conf.VALID_MINIMUM/2 - 1))
            bool = (0<int32(x)) && (int32(x)<=xmap) && (0<int32(y)) && (int32(y)<=ymap);
        end
    end
end



