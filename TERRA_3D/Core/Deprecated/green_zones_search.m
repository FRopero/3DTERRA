function [Vgz] = green_zones_search(t,R,dtm_map,gmap)
%  function [Vgz] = green_zones_search(t,R,dtm_map,gmap)
%  This function discover the set of vertices in green zone through which
%  the UAV can reach the target point 't', and then, it selects the first
%  one. However, in the case that the list of vertices in green zone was
%  empty, it returns to the user that this problem is unsolveable, because
%  there is no vertice near to the target point 't' through which the UAV
%  had enough energy to visit it.

%  Inputs:
%    t       - target point to reach
%    R       - Radius defining the maximum distance the UAV can travel in meters
%    dtm_map - Matrix with the map points (x,y,z) in meters.
%    gmap    - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    Vgz     - oficial vertice in green zone for this target point

start_node = struct('x',int32(t(1,1)),'y',int32(t(2,1)),'z',int32(t(3,1)),'f',int32(0)); %f is the level of the node

OPEN = start_node;
CLOSE = [];
Vgz = [];

while (~isempty(OPEN))
    
    [current_node, OPEN] = find_lowest(OPEN);
    
    %Delete current_node from OPEN
    [~,c] = size(OPEN);
    OPEN = OPEN(:,2:c);
       
    if (isGreen(current_node,gmap) && (current_node.x ~= start_node.x || current_node.y ~= start_node.y)) %GOAL = IS current_node IN GREEN ZONE?
        Vgz = [current_node.x;current_node.y;current_node.z];
        break
    end
    
    %Obtain promising successors
    [SUCS] = expandSuccessors(start_node,current_node,R,dtm_map,gmap);
    
    %Check if those successors has been visited yet
    [~,n_sucs] = size(SUCS);
    for i=1:n_sucs
        [in_closed] = is_visited(CLOSE, SUCS(i));
        if (~in_closed)
            [in_open] = is_visited(OPEN, SUCS(i));
            if (~in_open)
                node = SUCS(i);
                OPEN = [OPEN node];
            end
        end
    end
    
    %Put current_node in list of nodes currently visited CLOSE
    CLOSE = [CLOSE current_node];
end


end

function [OPEN] = deleteCurrent_deprecated(OPEN) 

L = [];
[~,c] = size(OPEN);

for i=1:c 
    if (OPEN(i).f ~= Inf)
        L = [L OPEN(i)];
    end
end

OPEN = L;

end

function [in_closed] = is_visited_deprecated(CLOSE, node)
    [~,n_visited] = size(CLOSE);
    in_closed = false;
    for i=1:n_visited
        if (CLOSE(i).x == node.x) && (CLOSE(i).y == node.y)
            in_closed = true;
            break
        end
    end

end

function [node, L] = find_lowest_deprecated(OPEN)
%This function order the OPEN list from the node with the lowest F value to
%the node with the highest F value

[~,co] = size(OPEN);
L = [];
lowest = 1;
[~,cl] = size(L);

while(cl<co)
    for j=1:co
        if (OPEN(j).f~=Inf)
            node = OPEN(j);
            lowest = j;
            break
        end
    end
    if (class(node)=='struct')
        for i=1:co
            if (OPEN(i).f~=Inf)
                if (OPEN(i).f < node.f)
                    node = OPEN(i); %Catch the lowest node
                    lowest = i;
                end
            end
        end

        OPEN(lowest).f = Inf;
        L = [L node]; %Return value
        node = Inf;
        [~,cl] = size(L);
    end
end

node = L(1); %Return value

end

function [L] = expandSuccessors(start_node,current_node,R,dtm_map,gmap)

L = [];
%8-positions to move from each node
new_node = struct('x',current_node.x,'y',current_node.y+1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x+1,'y',current_node.y+1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x+1,'y',current_node.y,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x+1,'y',current_node.y-1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x,'y',current_node.y-1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x-1,'y',current_node.y-1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x-1,'y',current_node.y,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end
new_node = struct('x',current_node.x-1,'y',current_node.y+1,'z',0,'f',current_node.f+1);
if (insideMapLimits(new_node,gmap))
    new_node.z = dtm_map(new_node.x,new_node.y);
    if (insideRadius(start_node,new_node,R))
        L = [L new_node];
    end
end

end

function [g] = isGreen(node,gmap)
    g = gmap(node.x,node.y) ~=0;
end

function [bool] = insideMapLimits(node,gmap)
    [xmap,ymap] = size(gmap);
    bool = (0<int32(node.x)) && (int32(node.x)<=xmap) && (0<int32(node.y)) && (int32(node.y)<=ymap);
end

function [bool] = insideRadius(start_node,node,R)
    d = sqrt( ((double(node.x)-double(start_node.x))^2) + ((double(node.y)-double(start_node.y))^2) + ((double(node.z)-double(start_node.z))^2) );
    bool = d<R;
end

function [node, L] = find_lowest(A)
    Afields = fieldnames(A);
    Acell = struct2cell(A);
    sz = size(Acell);

    % Convert to a matrix
    Acell = reshape(Acell, sz(1), []);      % Px(MxN)

    % Make each field a column
    Acell = Acell';                         % (MxN)xP

    % Sort by first field "f"
    Acell = sortrows(Acell, 4);

    % Put back into original cell array format
    Acell = reshape(Acell', sz);

    % Convert to Struct
    L = cell2struct(Acell, Afields, 1);
    node = L(1);
end

function [in_list] = is_visited(list, node)
    in_list = 0;
    if (~isempty(list))
        list = struct2table(list);
        list = list(:,1:2);
        node = struct2table(node);
        node = node(:,1:2);
        in_list = sum(ismember(list,node,'rows')) >=1;
    end
end