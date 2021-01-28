function [map_data, problem_params, dana_params] = compute_3DMap(dana_params, problem_params, cfgParams)
% function [dtm_map,gmap] = compute_3DMap(dtm_file,gmap_file)
%  This function read the DTM file and the RGB file to build the
%  exploration scenario of the instance
%  Inputs:
%    dana_params    - UGV Path Planning Parameters
%    problem_params - ECU-CSURP Parameters
%    cfgParams      - testing parameters
%  Outputs:
%    map_data       - 3D Map Data
%    problem_params - ECU-CSURP Parameters
%    dana_params    - UGV Path Planning Parameters

disp('Reading the 3D Map ...');

exe = ['java -Xmx10G -jar ..' cfgParams.slash 'Path-Planning' cfgParams.slash 'PathPlanning_old.jar -rsm'];

%Map Type
fname = strsplit(dana_params.map_file,cfgParams.slash);
fname = strsplit(fname{3},'.');
if (fname{2}=='IMG')
    map_type = 0;
elseif (fname{2}=='PNG')
    map_type = 1;
end

if (0 == map_type)
    %Configure Map
    dana_params.z = getSF(dana_params.map_file);
    %Check if it has been already readed, if not save it
    [is_precompiled,map_data,dana_params] = isCompiledMap(dana_params,map_type, cfgParams);
    % Read Map file
    if ~is_precompiled
        [map_data] = Read_DTM(dana_params);
        map_data.map_type = map_type;
        cmd = [' ', dana_params.map_file];
        cmd = [cmd,' ',num2str(dana_params.z),' ', num2str(dana_params.slope) ,' l'];
        cmd = strcat(exe,cmd);
        %Execute 3Dana to build the green zones map 'gmap'
        [status,~] = dos(cmd);
        
        if (status==0)
            gmap_file = strcat('reachabilityMap_',num2str(dana_params.slope));
            gmap_file = strcat(gmap_file,'.png');
            %Mode to precompiledMaps directory
            movefile(gmap_file,'..\3DMaps\PrecompiledMaps\');
            %gmap_file = strcat('..\3DMaps\PrecompiledMaps\',gmap_file);
            [gz_map] = Read_GZM(gmap_file,map_data.z_map,(map_data.map_conf.VALID_MINIMUM-1)/2,cfgParams);
            map_data.gz_map = gz_map;
        else
            disp('ERROR: There was an error executing PathPlanning.jar to build the Green Zones Map');
        end
    else
        
    end
elseif (1 == map_type)
    %Check if it has been already readed, if not save it
    [is_precompiled,map_data,dana_params] = isCompiledMap(dana_params,map_type, cfgParams);
    if (~is_precompiled)
        [map_data, dana_params] = Read_PNGM(dana_params,cfgParams);
        map_data.map_type = map_type;
        %dana_params.z = -1; 
    end
end

%Set Home's Altitude 
problem_params.Home(3,1) = map_data.z_map(problem_params.Home(1,1),problem_params.Home(2,1));
problem_params.Gp = problem_params.Home; %Gravity Point = Home

%Set the area of the map
[r,~] = size(map_data.z_map);
problem_params.Area = r; %To reduce the time of Dana computing!

if cfgParams.saveResults
    if ~is_precompiled
        fname = strsplit(dana_params.map_file,cfgParams.slash);
        fname = strsplit(fname{length(fname)},'.');
        fname = ['..' cfgParams.slash '3DMaps' cfgParams.slash 'PrecompiledMaps' cfgParams.slash fname{1} '.mat'];
        %Save recently readed .mat Map info
        save(fname,'map_data');
    end
end

if (cfgParams.printResults)
    vis = 'on';
else
    vis = 'off';
end
%Save altitudes map .fig recently created
fig = figure('visible',vis);
contourf(map_data.z_map',75)
mcolor = jet;
colormap(mcolor);
colorbar('eastoutside');
xlabel('X-Axis')
ylabel('Y-Axis')
zlabel('Z-Axis')
title('Altitudes Map');
axis equal;

if cfgParams.saveResults
    %Save altitudes map .png in the results dir
    fname = strsplit(dana_params.map_file,cfgParams.slash);
    fname = strsplit(fname{length(fname)},'.');
    path = [cfgParams.saveDir fname{1} '_altsMap.png'];
    saveas(fig,path);
end
end

%Read .PNG Maps
function [map_data, dana_params] = Read_PNGM(dana_params, cfgParams)
    fname = strsplit(dana_params.map_file,cfgParams.slash);
    fname = strsplit(fname{3},'.');
    output_f = ['..' cfgParams.slash '3DMaps' cfgParams.slash 'PrecompiledMaps' cfgParams.slash fname{1} '_z' num2str(dana_params.z) '_s' num2str(dana_params.slope) '.txt'];
    %New Map
    map_data = struct('z_map',[],'gz_map',[],'map_conf',[],'map_type',-1);
    
    f = imread(dana_params.map_file);
    [xmap,ymap] = size(f);

    ymap = ymap/3;
    z_map = [];
    gz_map = [];
    for i=1:xmap
        for j=1:ymap
            rgbmap(i,j,:) = uint8([f(i,j,1);f(i,j,2);f(i,j,3)]);
            if (f(i,j,1) >= 235 && f(i,j,2) >= 235 && f(i,j,3) >= 240) || (230 < f(i,j,1) && f(i,j,1) <= 255 && 200 < f(i,j,2) && f(i,j,2) <= 255 && 10 < f(i,j,3) && f(i,j,3) <= 150)
                rgbmap(i,j,:) = uint8([87;166;57]);
                z_map(i,j) = 1;
                gz_map(i,j) = 1;
            else
                rgbmap(i,j,:) = uint8([248;0;0]);
                z_map(i,j) = 9;
                gz_map(i,j) = 0;
            end
        end
    end
    
    %Print rgb map
    if (cfgParams.printResults)
        figure;
        image(rgbmap);
    end
    
    %Map configuration
    map_data.z_map = flipud(z_map)';
    map_data.gz_map = flipud(gz_map)';
    [xmap,ymap] = size(map_data.z_map);
    
    map_data.map_conf = struct('RECORD_BYTES',-1,'VALID_MINIMUM',8,'LINE_SAMPLES',xmap,'LINES',ymap,'SCALING_FACTOR',-1);

    %Generate Dana readable file
    fid=fopen(output_f,'w');
    for i=1:xmap
        for j=1:ymap
            if j==1
                fprintf(fid,'%d',9);
            else
                fprintf(fid,' %d',int64( map_data.z_map(i,j)));
            end
        end
        fprintf(fid,'\n');
    end
    status = fclose(fid);
  
  if (0 == status)
      dana_params.map_file = output_f;
  else
      disp('Error: There was a problem creating the streets map file.');
      dana_params.map_file = [];
  end
  
end

%Read DTM (.IMG) Maps (Mars Maps)
function [map_data] = Read_DTM(dana_params)

%New Map
map_data = struct('z_map',[],'gz_map',[],'map_conf',[],'map_type',-1);

fid = fopen (dana_params.map_file, 'r', 'ieee-le');
tline = fgetl(fid);

while ischar(tline)
   line = strsplit(tline);
   if (contains(tline, 'RECORD_BYTES'))
       header_size = str2double(line(3));
   else if (contains(tline, 'VALID_MINIMUM')) %#ok<SEPEX>
       valid_minimum = str2double(line(4));
      else if (contains(tline, 'LINE_SAMPLES')) %#ok<SEPEX>
              cols = str2double(line(4));
          else if (contains(tline, 'LINES')) %#ok<SEPEX>
              rows = str2double(line(4));
              else if (contains(tline, 'SCALING_FACTOR')) %#ok<SEPEX>
                 map_scale = str2double(line(4));
                  end
              end
          end
       end
   end
   tline = fgetl(fid);
end

map_data.map_conf = struct('RECORD_BYTES',header_size,'VALID_MINIMUM',valid_minimum,'LINE_SAMPLES',cols,'LINES',rows,'SCALING_FACTOR',map_scale);

fclose(fid);

fid = fopen (dana_params.map_file, 'r', 'ieee-le');
% Header at the top of the file
char(fread (fid, header_size, 'char')');  % throw it away

% Read the body
map_data.z_map = fread (fid, [cols,rows], 'float')';
fclose(fid);

MIN_POINT = min(min(map_data.z_map));

[r,c] = size(map_data.z_map);

for i=1:r
    for j=1:c
        if (MIN_POINT == map_data.z_map(i,j))
            map_data.z_map(i,j) = valid_minimum-1;
        end
    end
end

map_data.z_map = (map_data.z_map')*dana_params.z;

end
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

[gzmap] = imread(['..' cfgParams.slash '3DMaps' cfgParams.slash 'PrecompiledMaps' cfgParams.slash fname]);
%delete(fname);

%if (cfgParams.printResults)
%    fig = figure;
%    image(gzmap);
%    %imshow(flip(gzmap,1));
%    xlabel('X-Axis in DTM')
%    ylabel('Y-Axis in DTM')
%    zlabel('Z-Axis in DTM')
%    title('Green Zones Map');
%    axis equal;
%end

if (cfgParams.saveResults)
    fig = figure('visible','off');
    image(gzmap);
    xlabel('X-Axis in DTM')
    ylabel('Y-Axis in DTM')
    zlabel('Z-Axis in DTM')
    title('Green Zones Map');
    axis equal;
    path = [cfgParams.saveDir fname];
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

%Check if it has been already readed
function [is,data_map,dana_params] = isCompiledMap(dana_params,type, cfgParams)
   is = false;
   data_map = [];
   map_name = strsplit(dana_params.map_file,cfgParams.slash);
   map_name = strsplit(map_name{3},'.');
   name = map_name{1};
   map_name = ['..' cfgParams.slash '3DMaps' cfgParams.slash 'PrecompiledMaps' cfgParams.slash name '_z' num2str(dana_params.z) '_s' num2str(dana_params.slope) '.mat'];
   
   if 0 < exist(map_name,'file')
       load(map_name,'map_data');
       is = true;
       data_map = map_data;
       if (type == 1)
           dana_params.map_file = ['..' cfgParams.slash '3DMaps' cfgParams.slash 'PrecompiledMaps' cfgParams.slash name '_z' num2str(dana_params.z) '_s' num2str(dana_params.slope) '.txt'];
       end
   end
   
end

%Function to get the scaling factor from the dtm name file
function SF = getSF(dtm_file)
    st = sscanf(dtm_file,'%s');
    SF = -1;
    if (contains(st,'DTEEA'))
        SF = 4;
    elseif (contains(st,'DTEEB'))
        SF = 2;
    elseif (contains(st,'DTEEC'))
        SF = 1;
    elseif (contains(st,'DTEED'))
        SF = 0.5;
    end
        
end
