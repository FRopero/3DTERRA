% function [ ugv_path, ugv_distance ] = compute_3Dana( dtm_map, dana_params, ugv_path )
%  This function computes 3Dana to the ugv_path given as input.
%  Inputs:
%    dtm_map     - Matrix with the map points (x,y,z) in meters.
%    dana_params - Extra parameters to execute the path planning app
%    ugv_path    - Set of vertices to compute 3Dana
%    type        - Map Type: 0 = dtm_map(.IMG) ; 1 = png_map(.PNG)
%    cfgParams   - data parameters
%  Outputs:
%    ugv_path      - 3Dana solution for the UGV path
%    ugv_distance  - Distance in meters of the path

function [ ugv_path, ugv_distance ] = compute_3Dana( dtm_map, dana_params, ugv_path, type, cfgParams)

%TUBERIA VERSION: java -jar -Xmx4G D:\OneDrive\Universidad\Doctorado\Matlab\Path-Planning\PathPlanning.jar .sw D:\OneDrive\Universidad\Doctorado\Matlab\3DMaps\DTEEC_042666_1720_007696_1720_A01.IMG null 
exe =['java -Xmx10G -jar ..' cfgParams.slash 'Path-Planning' cfgParams.slash 'PathPlanning.jar'];
ugv_distance = 0;

% OJO!!: Para escenario de exploración
if (0==type)
    dtm_map = [' ', dtm_map];
    dtm_map = strcat (dtm_map,' null');
end
% OJO!!: Para escenario de calles
if (1==type)
    dtm_map = [' ',dtm_map,' ',dtm_map];
end

cmd = strcat(exe,dtm_map);
[r,~] = size(ugv_path);

% OJO!!: Para escenario de exploración
if (0==type)
    for i=1:r
        cmd = [cmd,' ',num2str(int64(ugv_path(i,1)))];
        cmd = [cmd,' ',num2str(int64(ugv_path(i,2)))];
    end
end
% OJO!!: Para escenario de calles
if (1==type)
    for i=1:r
        cmd = [cmd,' ',num2str(int64(ugv_path(i,2))-1)];
        cmd = [cmd,' ',num2str(int64(ugv_path(i,1))-1)];
    end
end

% OJO!!: Para escenario de exploración
if (0==type)
    cmd = [cmd,' -n',' ',num2str(dana_params.n),' -z',' ', num2str(dana_params.z),' ',num2str(dana_params.slope),' ',dana_params.path_file];
end

% OJO!!: Para escenario de calles
if (1==type)
    cmd = [cmd,' -n',' ',num2str(dana_params.n),' ',dana_params.path_file];%,' -z',' ', num2str(dana_params.z),' ',num2str(dana_params.slope),' ',dana_params.path_file];
end

%Execute 3dana
[status,~] = dos(cmd);
ugv_path=[];

%Read 3dana results
if (status==0)
    fileID = fopen(dana_params.path_file);
    text = textscan(fileID,'%s');
    
    %First, the ugv_distance
    if (char(text{1,1}(1,1))=='#p-length:')
        ugv_distance = str2double(cell2mat(text{1,1}(2,1)));
        if (char(text{1,1}(14,1))=='#nº-steps:')
            n_steps = str2double(cell2mat(text{1,1}(15,1)));
            n = 17;
            while(n_steps>0)
                % OJO!!: Para escenario de exploración
                if (0==type)
                    ugv_path = [ugv_path; str2double(cell2mat(text{1,1}(n,1))) str2double(cell2mat(text{1,1}(n+1,1))) str2double(cell2mat(text{1,1}(n+2,1)))];
                end
                
                % OJO!!: Para escenario de calles
                if (1==type)
                    ugv_path = [ugv_path;  str2double(cell2mat(text{1,1}(n+1,1)))+1 str2double(cell2mat(text{1,1}(n,1)))+1 str2double(cell2mat(text{1,1}(n+2,1)))];
                end
                
                n = n + 3;
                n_steps = n_steps - 1;
            end
        else
            disp('ERROR: Capturing the steps number.');
        end   
    else
        disp('ERROR: Capturing the ugv distance.');
    end
    fclose(fileID);
else
    %disp('ERROR: There was an error executing 3Dana.');
    ugv_path = -1;
end

end
