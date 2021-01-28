% function [solution] = Test_3D(problem_params, dana_params, cfgParams)
%  This function tests TERRA3D for a particular ECU-CSURP scenario
%  Inputs:
%    problem_params - ECU_CSURP Parameters
%    dana_params    - UGV Path Planning Parameters
%    map_data       - data info of the map
%    uav_data       - data info of the uav
%    cfgParams      - Configuration Parameters of the Test
%  Outputs:
%    solution       - struct with the ouput parameters      
function [solution] = Test_3D(problem_params, dana_params, map_data, uav_data, cfgParams)
    cnt = 1;
    solution = [];
    if (0~=map_data.gz_map(problem_params.Home(1,1),problem_params.Home(2,1))) %Check if the manual selected home location is in a green zone!
        for i=1:cfgParams.iterations
            %Create save dir for the results of this iteration
            itdir = [cfgParams.saveDir 'It-' num2str(cnt) cfgParams.slash];
            mkdir (itdir);

            % Generate Random 3D Scenario
            [problem_params.T] = scene_generator(problem_params, map_data, cfgParams, itdir);

            disp(['Computing solution for Scenario ' num2str(cnt) ' ...']);
            tic
            [data_sol, path_sol, figures] = TERRA3D(cfgParams, problem_params, map_data, dana_params, uav_data);
            tc = toc;
            
            %Saving
            if (cfgParams.saveResults)
               [solution] = saveResults(cfgParams,tc,data_sol,path_sol,problem_params,map_data,dana_params,uav_data,figures,itdir);
            end

            disp(['Execution ' num2str(cnt) ' completed.']);
            cnt = cnt + 1;
        end

    else
        disp('Error: The Home Point must be located on a green zone!')
    end

    %Simulation Stage
    if (cfgParams.Vrep)
        MainLoop(path_sol(1).path_solution);
    end

end


%This function stacks the iteration output to the general output, and also, save
%the iteration output
function [solution] = saveResults(cfgParams,tc,data_sol, path_sol,problem_params,map_data,dana_params,uav_data,figures, itdir)
    disp('Saving results ...');
    
    % Init solution struct
    %  cT      - array of the computational time taken by TERRA to compute a solution
    %  dataRes - array of the computational results of the TERRA execution (see definition in TERRA3D.m)
    %  pathRes - array of the UGV-UAV path planning solution of TERRA
    solution = struct('cT',[],'dataRes',[],'pathRes',[]);
    
    %Stacking
    solution.cT = tc;
    solution.dataRes =struct('It',data_sol);
    solution.pathRes = struct('It',path_sol);
    
    %Saving Data
    fpath = [itdir 'data.mat'];
    save(fpath,'tc','data_sol','path_sol','problem_params','map_data','dana_params','uav_data','cfgParams');
    
    %Saving Figures
    if (~isempty(figures))
        [~,c] = size(figures);
        for i=1:c
            name = [itdir figures(i).Name '.fig'];
            figure(figures(i));
            savefig(name);
        end
    end
    
    if (~cfgParams.printResults) 
        close all;
    end
   
end



