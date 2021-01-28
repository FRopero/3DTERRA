function [ path_sol ] = build_matrix_solution_3D( ugv_path, uav_path )
%%% Build Final Solution Path for the UGV and UAV
path_sol = [];
for i=1:length(ugv_path)
    path_sol = [path_sol struct('c_ugv',ugv_path(i,:),'c_uav',[])];
end

%Insert c_uav to the final path solution
[~,r] = size(path_sol);
[~,u] = size(uav_path);
for i=1:r
    for j=1:u
        if (path_sol(i).c_ugv(1)==uav_path(j).Coordinates(1,1) && path_sol(i).c_ugv(2)==uav_path(j).Coordinates(2,1))
            path_sol(i).c_uav = uav_path(j).Coordinates;
        end
    end
end



end

