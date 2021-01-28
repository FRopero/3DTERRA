%  function [Vgz] = green_zones_ordered_search(t,R,gmap)
%  This function roams around the target point 't' in a slail mode from the
%  outbounds to the target point, aiming to find a green zone vertice
%  inside the radius 'R' of the target point 't'.

%  Inputs:
%    t       - target point to reach
%    R       - Radius defining the maximum distance the UAV can travel in meters
%    gmap    - Matrix with the green zones defined. X=green zone 0=Otherwise
%  Outputs:
%    Vgz     - oficial vertice in green zone for this target point
function [Vgz] = green_zones_ordered_search(t,R,gmap)

z_square = 0;
Vgz = [];
found = false;

for i=0:R-1
    y_square = int64(t(2,1))-R+i;
    x_square = int64(t(1,1));
    total_squares = mod(t(2,1),y_square)*4;
    wide_quadrant = total_squares/4;
    
    for l=0:total_squares-1
        
        if (insideMapLimits(x_square,y_square,gmap))
            if isGreen(x_square,y_square,gmap)
                z_square = gmap(x_square,y_square);
                if insideRadius([x_square;y_square;z_square],t,R)
                    found = true;
                    break
                end
            end
        end
        
        if (l<wide_quadrant)
                %disp('Q1')
                x_square = x_square+1;
                y_square = y_square+1;
        elseif (wide_quadrant<=l && l<wide_quadrant*2)
                %disp('Q2')
                x_square = x_square-1;
                y_square = y_square+1;
        elseif (wide_quadrant*2<=l && l<wide_quadrant*3)
                %disp('Q3')
                x_square = x_square-1;
                y_square = y_square-1;
        elseif (wide_quadrant*3<=l && l<wide_quadrant*4)
                %disp('Q4')
                x_square = x_square+1;
                y_square = y_square-1;
        end
    end
    
    if (found)
        Vgz = [x_square;y_square;z_square];
        break
    end
end

if (~found)
    if (insideMapLimits(int64(t(1,1)),int64(t(2,1)),gmap))
        if isGreen(int64(t(1,1)),int64(t(2,1)),gmap)
            z_square = gmap(int64(t(1,1)),int64(t(2,1)));
            Vgz = [int64(t(1,1));int64(t(2,1));z_square];
        end
    end
end

end

function [g] = isGreen(x,y,gmap)
    g = gmap(x,y) ~=0;
end

function [bool] = insideMapLimits(x,y,gmap)
    [xmap,ymap] = size(gmap);
    bool = (0<int32(x)) && (int32(x)<=xmap) && (0<int32(y)) && (int32(y)<=ymap);
end

function [bool] = insideRadius(vertice,t,R)
    d = sqrt( ((double(t(1,1))-double(vertice(1,1)))^2) + ((double(t(2,1))-double(vertice(2,1)))^2) + ((double(t(3,1))-double(vertice(3,1)))^2) );
    bool = d<R;
end