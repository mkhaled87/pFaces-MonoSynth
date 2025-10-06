%
% closedloop.m
%
% created on: 18.01.2018
%     author: M.Khaled
%
% you need to run pFaces with the config file 'pfaces.cfg' first 
%
function closedloop()
    
    % cleaning
    close all;
    mainFig = figure;
    figure(mainFig);

    % add the path of common subroutines
    include = '../common/matlab/';
    addpath(include);

    % loading the controller data file and reading the dynamics
    boolCloseAnyOtherFiles = true;
    exampleDataFile = DataFile('vehicle.raw', boolCloseAnyOtherFiles, true);
    symbolicDynamics = ReadDynamicsSymbolic(exampleDataFile, [], [], {'(float)'});

    % start the simulation
    initial_state = [0.5 1.0 pi/2];
    SimulateClosedLoop2D(exampleDataFile, @sys_ode, 1, 2, initial_state);

    % remove the path from search subroutines
    rmpath(include);

    % The ODE of the system as a nested function
    function dxdt = sys_ode(t,x,u)

        persistent f_xx1;
        persistent f_xx2;
        persistent f_xx3;
        
        if isempty(f_xx1) 
            f_xx1 = matlabFunction(symbolicDynamics(1)); 
        end
        if isempty(f_xx2) 
            f_xx2 = matlabFunction(symbolicDynamics(2)); 
        end
        if isempty(f_xx3) 
            f_xx3 = matlabFunction(symbolicDynamics(3)); 
        end
        
        dxdt = zeros(3, 1);
        dxdt(1) = f_xx1(u(1), u(2), x(3));
        dxdt(2) = f_xx2(u(1), u(2), x(3));
        dxdt(3) = f_xx3(u(1), u(2));
    end
end

