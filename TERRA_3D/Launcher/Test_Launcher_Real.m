%                        -> 3D TERRA Launcher <-                          %

% This script launchs an execution of TERRA-3D over a random generated
% scenario to solve the Energy Constrained UAV and Charging Station UGV 
% Routing Problem (ECU-CSURP).
% For more information, please refer to the paper:
%   Ropero, F., Muñoz, P., & R-Moreno, M. D. TERRA: A path planning 
%   algorithm for cooperative UGV–UAV exploration. Engineering Applications 
%   of Artificial Intelligence, 78, 260-272, 2019.

% System Settings
if isunix
    slash='/';
elseif ispc
    slash='\';
end

% Configuration of the testing parameters
%  iterations   = nº of executions with different scenarios
%  printResults = LoL
%  saveResults  = xd
%  Vrep         = to launch the V-REP simulation
%  saveDir      = o.O
format shortg
[c, ~] = clock;
dir = ['Results' slash 'Test-' num2str(c(3)) '.' num2str(c(2)) '.' num2str(c(1)) '.' num2str(c(4)) '.' num2str(c(5)) slash];
cfgParams = struct('iterations',1,'printResults',true,'saveResults',true,'Vrep',false,'saveDir',dir,'slash',slash);

%Create directory if saveresults = true
if (cfgParams.saveResults)
    [status, msg, msgID] = mkdir(dir);
end

% ECU_CSURP Parameters
%  T    = Target Points of the scenario
%  N    = Number of target points
%  R    = Farthest distance the UAV can travel
%  D    = Number of groups for the random map generator
%  Home = Home location (DTM_1535_A01 = [548;600;-1] ; PNG Torres = [215;558;-1] ; PNG Madrid = [253,167,-1])
%  Area = 3d map area
%  Gp   = gravitational point
problem_params = struct('T',[],'R',300,'N',5,'D',5,'Home',[215;558;-1],'Area',600,'Gp',[1;1;1]);

% UGV's TSP Genetic Algoritm Parameters (Current Used = Id. 2 from TERRA paper)
%  popSize     = size of the population of chromosomes.
%  tournaments = number of chromosomes to select for the tournament selection
%  mutOper     = mutation operator (1 = Flip; 2 = Swap; 3 = Slide)
%  mutRate     = mutation rate from 0.0 to 1.0 (0-100%)
%  crossOper   = crossover operator (1 = OX; 2 = CX; 3 = CBX)
%  eliteP      = elitism selection from 0 to 100%
ugv_tsp = struct('popSize',430,'tournaments',9,'mutOper',2,'mutRate',0.06,'crossOper',1,'eliteP',2.7);

% UGV Path Planning Parameters
%  n         = foo
%  z         = scale factor
%  slope     = max. terrain slope that the UGV is able to cross
%  path_file = output file of the path planning application
%  map_file  = input map. Admited formats: .PNG, .IMG ,e.g: DTEED_030808_1535_031230_1535_A01.IMG or mapa-torres.PNG
%  ugv_tsp   = UGV's TSP Genetic Algoritm Parameters
dana_params = struct('n',0,'z',-1,'slope',20,'path_file','ugv_path','map_file',['..' slash '3DMaps' slash 'mapa-torres.PNG'],'ugv_tsp',ugv_tsp);

% UAV Path Planning Parameters
%  R = farther distance the UAV can travel
%  Z = fixed altitude of the UAV's path
uav_data = struct('R',problem_params.R,'Z',5);

% 3D Map Data
%  z_map    = map of altitudes
%  gz_map   = map with the green zones defined. X=green zone (altitude) 0=Otherwise
%  map_conf = configuration of the map
%  map_type = map type. 0=dtm 1=png/txt
map_data = struct('z_map',[],'gz_map',[],'map_conf',[],'map_type',1);
map_data.map_conf.VALID_MINIMUM = -2;
height = 600; %cm
width = 400;  %cm

for r=1:height
    for c=1:width
        map_data.gz_map(r,c)=1;
        map_data.z_map(r,c)=1;
    end
end

%Compute 3D Map
[map_data, problem_params, dana_params] = compute_3DMap(dana_params, problem_params, cfgParams);

%Launch TERRA3D
[solution] = Test_3D(problem_params, dana_params, map_data, uav_data, cfgParams);

%Other Parameters
%T = [2139,4249;4024,5399;4449,5549;4699,5749;4799,6449;4832,6469;4869,6489]; Gusev Crater
%T = [862,725;936,287;1066,547;573,740;600,1400;2300,2000;1300,2300]; %Default Map
%T = [2820,990;1490,2470;1100,2820]; Gale Crater
%Home = [3450;240]; Gale Crater
%Home = [1999;4419]; Gusev Crater
