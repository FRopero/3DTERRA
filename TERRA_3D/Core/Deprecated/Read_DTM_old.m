function [map_data, dana_params] = Read_DTM(dana_params)

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


