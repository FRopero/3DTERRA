function [gmap] = Read_GZM(fname, z_map, valid_min, cfgParams)
%  function [gmap] = Read_GZM(fname, map, valid_min)
%  This function read the png file to detect the green zones.
%  Inputs:
%    fname      - filename of the green zones map
%    z_map      - altitudes map
%    valid_min  - min value for the map boundaries
%    cfgParams      - testing parameters
%  Outputs:
%    gmap    - set of vertices in meters


[gzmap] = imread(fname);
delete(fname);

if (cfgParams.printResults)
    fig = figure;
    image(gzmap);
    xlabel('X-Axis in DTM')
    ylabel('Y-Axis in DTM')
    zlabel('Z-Axis in DTM')
    title('Green Zones Map');
    axis equal;
end

if (cfgParams.saveResults)
    fig = figure('visible','off');
    image(gzmap);
    xlabel('X-Axis in DTM')
    ylabel('Y-Axis in DTM')
    zlabel('Z-Axis in DTM')
    title('Green Zones Map');
    axis equal;
    path = ['..\' cfgParams.saveDir fname];
    saveas(fig,path);
end

%RGB - gzmap (X,Y)x3 - R = 1 : G = 2 : B = 3
z_map = z_map';
[xmap,ymap] = size(z_map);
for i=1:xmap
    for j=1:ymap
        R = gzmap(i,j,1);
        G = gzmap(i,j,2);
        if (z_map(i,j)~=valid_min && G == 255)
            gmap(i,j) = z_map(i,j);
        else
            gmap(i,j) = 0;
        end
    end
end

gmap = gmap';

end

