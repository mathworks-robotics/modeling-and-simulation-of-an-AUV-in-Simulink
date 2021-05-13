classdef auvPathPlanner < matlab.System
    %This system is used to plan paths using RRT or aStar planners
    
    properties(Nontunable)                      %Nontunable parameters
        occMap = 'uavMapCityBlock.mat'             %Occupany map for planner state validator
        maxRollAngle = pi/300                       %Maximum roll angle
        airSpeed = 1                               %Airspeed of UAV
        flightPathAngleLimit = [-0.1 0.1]          %Minimum and maximum flight path angles
        bounds = [-250 250;-250 250 ;-250 250; -pi pi]  %State space bounds
        searchCoordinates = [100 100]              %AStar search grid size
        searchFOV = 5                              %AStar search field of view
        bouyPostions =  [50,50,50,150,150,100]                           %Postion of bouys
        bouyCoverage = 150                         %Coverage distance for bouys
    end
    
    properties(DiscreteState)
    end
    
    properties (Access = private) %Pre-computed constants
        planner
        planner2                  %RRT  planners
        stateSpace                %UAV state space
        stateSpace2               %SE3 state space
        stateValidator            %UAV state validator
        stateValidator2           %SE3 state validator
        occMapAxis                %3D occupancy map axis
    end
    
    methods(Access = protected)
        
        function [waypoints] = stepImpl(obj, startPose, goalPose, pathType)
            % % Execute Planner Stage 2, Grid Search
            if (pathType)
                [xCoord,yCoord,zCoord] = inspectionPlanner(obj.searchCoordinates(1), obj.searchCoordinates(2),...
                    startPose(1), startPose(2), startPose(3),obj.searchFOV);
                waypoints = [xCoord',yCoord',zCoord',zeros(length(xCoord),1)];
            else
                if(isequal(startPose(1),goalPose(1)) || isequal(startPose(2),goalPose(2)) || isequal(startPose(3),goalPose(3)))
                    [path,~] = plan(obj.planner2,[startPose(1:3) 1 0 0 0],[goalPose(1:3) 1 0 0 0]);
                    interpolatedPathObj = copy(path);
                    interpolate(interpolatedPathObj,1000);
                    
                    % smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);
                    if (path.NumStates > 3)
                        smoothWaypointsObj = exampleHelperUAVPathSmoothingMod(obj.stateSpace2,obj.stateValidator2,path);
                        interpolatedSmoothWaypoints = copy(smoothWaypointsObj);
                    else
                        interpolatedSmoothWaypoints = interpolatedPathObj;
                    end
                    waypoints = interpolatedSmoothWaypoints.States;
                       %waypoints = path.States;
                else
                    [~,~,~,path] = exampleExecutePlanner(obj.planner,obj.stateSpace,obj.stateValidator,startPose,goalPose);
                    waypoints = path.States;
                end
            end
            
%             if (~pathType)
%                 scatter3(obj.occMapAxis,startPose(1), startPose(2), startPose(3), 30, "red", "filled");
%             end
%             scatter3(obj.occMapAxis,goalPose(1), goalPose(2),  goalPose(3), 30, "red", "filled");
%             plot3(obj.occMapAxis,waypoints(:,1), waypoints(:,2),...
%                 waypoints(:,3), "LineWidth",2,"Color","g");
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.refresh();
        end
        
        function resetImpl(~)
        end
        
        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [20000 4];
        end
        
        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
        end
        
        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
        end
        
        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = false;
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", 0.5);
        end
        
    end
    
    methods(Access = protected, Static)
        
        %         function simMode = getSimulateUsingImpl
        %             % Return only allowed simulation mode in System block dialog
        %             simMode = 'Interpreted execution';
        %         end
    end
    
    methods (Access = private)
        function refresh(obj)
            
            %load occupancy map
            figure
            mapData = load(obj.occMap, "omap");
            omap = mapData.omap;
            obj.occMapAxis = show(omap);
            title(obj.occMapAxis, 'Planning Map')
            hold on
            omap.FreeThreshold = omap.OccupiedThreshold;
            inflate(omap, 1);
            %CoveragePlot(obj.bouyPostions,obj.bouyCoverage);
            
            % Define State Space and State Validator
            obj.stateSpace = ExampleHelperUAVStateSpace("MaxRollAngle",obj.maxRollAngle,...
                "AirSpeed",obj.airSpeed,...
                "FlightPathAngleLimit",obj.flightPathAngleLimit,...
                "Bounds",obj.bounds);
            obj.stateValidator = validatorOccupancyMap3D(obj.stateSpace,"Map",omap);
            obj.stateValidator.ValidationDistance = 0.1;
            
            % Setup Planner
            obj.planner = plannerRRT(obj.stateSpace,obj.stateValidator);
            obj.planner.MaxConnectionDistance = 50;
            obj.planner.MaxIterations = 1000;
            obj.planner.GoalBias = 0.1;
            obj.planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);
            
            % Define Second State Space and State Validator
            obj.stateSpace2 = stateSpaceSE3([-20 220;-20 220;-10 100; inf inf; inf inf; inf inf; inf inf]);
            obj.stateValidator2 = validatorOccupancyMap3D(obj.stateSpace2,"Map",omap);
            obj.stateValidator2.ValidationDistance = 1;
            
            % Setup Second Planner
            obj.planner2 = plannerRRT(obj.stateSpace2,obj.stateValidator2);
            obj.planner2.MaxConnectionDistance = 1;
            obj.planner2.MaxIterations = 1000;
            obj.planner2.GoalBias = 0.1;
            obj.planner2.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);
            
        end
    end
end