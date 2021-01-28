%function [ rte, dist, stops] = search_uav_3Dpath( subpath, R)
%This function is a modified search algorithm of the well-known A* algorithm. The goal of
%this function is to find out which is the SHORTEST PATH for the UAV
%starting in the start_node{path_x(1), path_y(1)} and finishing in the
%same node, visiting only one time each waypoint. Thus, it is a
%modification of the TSP, with the restriction of the UAV's energy (R).
%subpath = {V_init,t1,t2,t3};

%This algorithm is going to create a graph non-directed to achieve its
%goal. Each node of the graph is a matlab struct like this:
% struct {
%   struct parent
%   coordinates x, y
%   g = dE (dE = Euclidean distance accumulated from Home) 
%   k = dE accumulated from the last stop at Home
%   h = nº of remaining visited nodes
%   f = g + h
% }
function [ rte, dist, stops] = search_uav_3Dpath( subpath, R)

V_init = subpath(:,1);
stops = 0;

%N = nº total of nodes
[~,N] = size(subpath);
h = N-1;

%Start Node: g = 0 : k = 0 : h = N-1 : f = g + h
root_node = struct('parent',-1,'x',V_init(1,1),'y',V_init(2,1),'g',0,'k',0,'h',h,'f',h);
sol_found = false;
dist = 0;
rte = [];

% The ordered list of currently discovered nodes that are not evaluated yet.
% Initially, only the start node is known.
OPEN = root_node;

while (~isempty(OPEN))
    
    [current_node, OPEN] = find_lowest(OPEN);
    
    %Delete current_node from OPEN
    OPEN(1).f = Inf; %OPEN(1) = current_node
    [OPEN] = deleteCurrent(OPEN);
    
    %%Goal Node is HOME with h=0
    if ((current_node.x == V_init(1,1)) && (current_node.y == V_init(2,1)) && (current_node.h == 0))
        %disp('UAV Subpath solution found!');
        sol_found = true;
        break
    end
    
    [SUCS] = expand_graph(current_node, subpath, R, root_node); %List of successors
    
    OPEN = [OPEN SUCS];
end

if (sol_found)
    [rte, dist] = reconstruct_path(current_node, rte, dist, subpath);
    [~,c] = size(rte);
    for i=2:c-1
        if (rte(i)==1)
            stops = stops + 1;
        end
    end
else
    %disp('UAV Subpath solution NOT found!');
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

function [rte, dist] = reconstruct_path(current_node, rte, dist, path)
[~,c] = size(path);

for i=1:c
    if (current_node.x == path(1,i) && current_node.y == path(2,i))
        rte = [i rte];
        if (class(current_node.parent)=='struct')
            dist = dist + sqrt(((current_node.x - current_node.parent.x)^2) + ((current_node.y - current_node.parent.y)^2));
            [rte, dist] = reconstruct_path(current_node.parent, rte, dist, path);
            break;
        end
    end
end

end

function [node, L] = find_lowest(OPEN)
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

function [L] = expand_graph(current_node, path, Radius, root_node)

[~,c] = size(path);
L = [];
home = [root_node.x;root_node.y];

for i=1:c
    suc = [path(1,i);path(2,i)];
    %If current node is not the root of the tree, and the successor it is
    %not this same point
    if (current_node.x ~= suc(1,1) && current_node.y ~= suc(2,1))
        %Check if node(i) is a relative of the current node
        if (suc(1,1)==root_node.x && suc(2,1)==root_node.y)
            is_rel = false;
        else
            [is_rel] = is_relative(current_node, suc);
        end
        if (~is_rel)
            if (current_node.x == root_node.x && current_node.y == root_node.y)
                k = sqrt(((path(1,i)-current_node.x)^2) + ((path(2,i)-current_node.y)^2));
            else
                k = current_node.k + sqrt(((path(1,i)-current_node.x)^2) + ((path(2,i)-current_node.y)^2));
            end
            g = current_node.g + sqrt(((path(1,i)-current_node.x)^2) + ((path(2,i)-current_node.y)^2));
            d_tohome = sqrt(((path(1,i)-root_node.x)^2) + ((path(2,i)-root_node.y)^2));
                     
            %The UAV has enough energy to go this successor node and return to home
            if (Radius*2 >= vpa(k + d_tohome))
                if (suc==home)
                    h = current_node.h;
                else
                    h = current_node.h - 1;
                end
                %No relative, so, we add the successor to the OPEN List
                suc = struct('parent',current_node,'x',path(1,i),'y',path(2,i),'g',g,'k',k,'h',h,'f',g+h);
                L = [L suc];
            end
        end
    end
end

end

function [is_rel] = is_relative(current, suc)
%This function search if a node has been already visited in the path
if (class(current)=='struct')
    if (current.x == suc(1,1) && current.y == suc(2,1))
        is_rel = true;
    else
        [is_rel] = is_relative(current.parent,suc);
    end
else
    %The last, it will be current.parent == -1
    is_rel = false;
end

end
