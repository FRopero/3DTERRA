function [Vgz] = green_zones_search(best_v,cT,R,gmap)
%  function [Vgz] = green_zones_search(v,t,R,gmap)
%  This function checks if the vertice 'v' is inside a green zone defined
%  in gmap. If it finds out that 'v' is in a green zone, it returns Vqz = v
%  and in_green = true. If not, it executes a local search from this v
%  around the 'map'
%  Inputs:
%    best_v - nearest vertice covering the set of target points 'cov_T'
%    cT     - set of target points covered by best_v
%    R      - Radius defining the maximum distance the UAV can travel in meters
%    gmap   - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    Vgz    - oficial vertice in green zone for this target points

Vgz = -1;
[xmap,ymap] = size(gmap);
OPEN = [];
CLOSE = [];
best_v(3,1)=gmap(int64(best_v(1,1)),int64(best_v(2,1)));
[~,f_start] = distance_condition(best_v,cT,R);
start_node = struct('parent',-1,'x',int64(best_v(1,1)),'y',int64(best_v(2,1)),'z',best_v(3,1),'f',f_start); % f= sum of 'x' distances from the vertice 'best_v' to every target point in cov_T
OPEN = start_node;

%The search it is finished when there is not more nodes. That means wheter 
%some green zone is been reached or not
while (~isempty(OPEN))
    %Dequeue current_node from the FIFO queue 'OPEN'
    current_node = OPEN(1);
    OPEN(1).f = Inf; %OPEN(1) = current_node
    [OPEN] = deleteCurrent(OPEN);
    
    if (0~=gmap(current_node.x,current_node.y)) %GOAL = IS current_node IN GREEN ZONE?
        Vgz = best_v;
        break
    end
    
    %Obtain promising successors
    [SUCS] = expand_graph(current_node, xmap, ymap, cT, R, gmap); %List of successors
    
    %Check if those successors has been visited yet
    [~,n_sucs] = size(SUCS);
    for i=1:n_sucs
        [in_closed] = is_visited(CLOSE, SUCS(:,i));
        if (~in_closed)
            node = struct('parent',current_node,'x',SUCS(1,i),'y',SUCS(2,i),'z',SUCS(3,i),'f',SUCS(4,i));
            OPEN = [OPEN node];
        end
    end
    
    %Put current_node in list of nodes currently visited CLOSE
    CLOSE = [CLOSE current_node];
end

end

function [OPEN] = deleteCurrent(OPEN) 

L = [];
[~,c] = size(OPEN);

for i=1:c 
    if (OPEN(i).f ~= Inf)
        L = [L OPEN(i)];
    end
end

OPEN = L;

end

function [in_closed] = is_visited(CLOSE, node)
    [~,n_visited] = size(CLOSE);
    in_closed = false;
    for i=1:n_visited
        if (CLOSE(1,i).x == node(1,1)) && (CLOSE(1,i).y == node(2,1))
            in_closed = true;
            break
        end
    end

end

function [L] = expand_graph(current_node, xmap, ymap, cT, R, gmap)

    %8-positions to move from each node
    L = [];
    SUCS = [];
    SUCS = [SUCS [current_node.x;current_node.y+1;gmap(current_node.x,current_node.y+1)]];
    SUCS = [SUCS [current_node.x+1;current_node.y+1;gmap(current_node.x+1,current_node.y+1)]];
    SUCS = [SUCS [current_node.x+1;current_node.y;gmap(current_node.x+1,current_node.y)]];
    SUCS = [SUCS [current_node.x+1;current_node.y-1;gmap(current_node.x+1,current_node.y-1)]];
    SUCS = [SUCS [current_node.x;current_node.y-1;gmap(current_node.x,current_node.y-1)]];
    SUCS = [SUCS [current_node.x-1;current_node.y-1;gmap(current_node.x-1,current_node.y-1)]];
    SUCS = [SUCS [current_node.x-1;current_node.y;gmap(current_node.x-1,current_node.y)]];
    SUCS = [SUCS [current_node.x-1;current_node.y+1;gmap(current_node.x-1,current_node.y+1)]];

    for i=1:8
        [cond,f] = distance_condition(SUCS(:,i),cT,R);
        if (cond) %Every target point is covered?
            SUCS(4,i) = f;
            if ((0<=int32(SUCS(1,i))) && (int32(SUCS(1,i))<=xmap) && (0<=int32(SUCS(2,i))) && (int32(SUCS(2,i))<=ymap)) % The vertice is inside the map boundaries
                L = [L SUCS(:,i)];
            end  
        end
    end

end

function [cond,f] = distance_condition(node,cov_T,R)
% Function to check if the target points in 'cov_T' are still covered by
% the vertice 'node'. It returns the distance sum of all the distances
% between the vertice and the target points.
    [~,c] = size(cov_T);
    cond = true;
    f=0;
    for i=1:c
        d = sqrt( ((double(node(1,1))-cov_T(1,i))^2) + ((double(node(2,1))-cov_T(2,i))^2) + ((double(node(3,1))-cov_T(3,i))^2) );
        if (d > R)
            cond = false;
            f=0;
        else 
            f = f + d;
        end
    end

end


