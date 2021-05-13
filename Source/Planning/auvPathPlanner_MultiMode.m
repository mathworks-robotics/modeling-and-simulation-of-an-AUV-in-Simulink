classdef auvPathPlanner_MultiMode < matlab.System
    %This system is used to plan paths using RRT or aStar planners
    
    properties(Nontunable)                      %Nontunable parameters
        occMap = 'uavMapCityBlock.mat'             %Occupany map for planner state validator
        maxRollAngle = pi/300                       %Maximum roll angle
        airSpeed = 1                               %Airspeed of UAV
        flightPathAngleLimit = [-0.1 0.1]          %Minimum and maximum flight path angles
        bounds = [-250 250;-250 250 ;-250 250; -pi pi]  %State space bounds
        searchCoordinates = [100 100]              %AStar search grid size
        searchFOV = 10                              %AStar search field of view
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
        omap
        occMapAxis                %3D occupancy map axis
    end
    
    methods(Access = protected)
        
        function [waypoints, planningStatus] = stepImpl(obj, startPose, goalPose, pathType)
            % % Execute Planner Stage 2, Grid Search
            planningStatus = 0;
            
            if (pathType==1)
                %                 [xCoord,yCoord,zCoord] = inspectionPlanner(obj.searchCoordinates(1), obj.searchCoordinates(2),...
                %                     startPose(1), startPose(2), startPose(3),obj.searchFOV);
                load('gridPath.mat');
                waypoints =zeros(length(xCoord),4);
                
                for i=1:10:length(xCoord)-1
                    if (xCoord(i)==xCoord(i+1))
                        waypoints(i,:) = [xCoord(i),yCoord(i),zCoord(i),1.5];
                    elseif    (yCoord(i)==yCoord(i+1))
                        waypoints(i,:) = [xCoord(i),yCoord(i),zCoord(i),0];
                    end
                end
                waypoints(length(xCoord)) = waypoints(length(xCoord)-1);
                [r,~] = find(waypoints);
                waypoints= waypoints(r,:);
                
                
            elseif (pathType==2)
                %if(isequal(startPose(1),goalPose(1)) || isequal(startPose(2),goalPose(2)) || isequal(startPose(3),goalPose(3)))
                
                path = navPath(stateSpaceSE3);
                while(isempty(path.States))
                    [path,~] = plan(obj.planner2,[startPose(1:3) 1 0 0 0],[goalPose(1:3) 1 0 0 0]);
                end
                
                
                interpolatedPathObj = copy(path);
                if (interpolatedPathObj.NumStates<=50)
                    interpolate(interpolatedPathObj,100);
                end
                %smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);
                if (path.NumStates > 3)
                    smoothWaypointsObj = exampleHelperUAVPathSmoothingMod(obj.stateSpace2,obj.stateValidator2,path);
                    interpolatedSmoothWaypoints = copy(smoothWaypointsObj);
                else
                    interpolatedSmoothWaypoints = interpolatedPathObj;
                end
                interpolate(interpolatedSmoothWaypoints,100);
                waypoints = interpolatedSmoothWaypoints.States;
                %waypoints = interpolatedPathObj.States;
                
            elseif (pathType==3)
                [~,~,~,path] = exampleExecutePlanner(obj.planner,obj.stateSpace,obj.stateValidator,startPose,goalPose);
                waypoints = path.States;
                
            end
            
            %             obj.occMapAxis = show(obj.omap);
            if (pathType~=1)
                scatter3(obj.occMapAxis,goalPose(1), goalPose(2),  goalPose(3), 30, "red", "filled");
            end
            scatter3(obj.occMapAxis,startPose(1), startPose(2), startPose(3), 30, "red", "filled");
            scatter3(obj.occMapAxis,goalPose(1), goalPose(2),  goalPose(3), 30, "red", "filled");
            plot3(obj.occMapAxis,waypoints(:,1), waypoints(:,2),...
                waypoints(:,3), "LineWidth",2,"Color","g");
            %             if (pathType~=1)
            %                 plot3(obj.occMapAxis,waypoints(:,1), waypoints(:,2),...
            %                 waypoints(:,3), "LineWidth",2,"Color","g");
            %             else
            %             plot3(obj.occMapAxis,waypoints(:,1), waypoints(:,2),...
            %                 waypoints(:,3),'.',"Color","g");
            %             end
            planningStatus = 1;
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.refresh();
        end
        
        function resetImpl(~)
        end
        
        function [out1,out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [20000 7];
            out2 = [1];
        end
        
        function [out1,out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
        end
        
        function [out1,out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
        end
        
        function [out1,out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = false;
            out2 = true;
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
            obj.omap = mapData.omap;
            obj.occMapAxis = show(obj.omap);
            title(obj.occMapAxis, 'Planning Map')
            hold on
            obj.omap.FreeThreshold = obj.omap.OccupiedThreshold;
            inflate(obj.omap, 1);
            %CoveragePlot(obj.bouyPostions,obj.bouyCoverage);
            
            % Define State Space and State Validator
            obj.stateSpace = ExampleHelperUAVStateSpace("MaxRollAngle",obj.maxRollAngle,...
                "AirSpeed",obj.airSpeed,...
                "FlightPathAngleLimit",obj.flightPathAngleLimit,...
                "Bounds",obj.bounds);
            obj.stateValidator = validatorOccupancyMap3D(obj.stateSpace,"Map",obj.omap);
            obj.stateValidator.ValidationDistance = 0.1;
            
            % Setup Planner
            obj.planner = plannerRRT(obj.stateSpace,obj.stateValidator);
            obj.planner.MaxConnectionDistance = 50;
            obj.planner.MaxIterations = 1000;
            obj.planner.GoalBias = 0.1;
            obj.planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 5);
            
            % Define Second State Space and State Validator
            obj.stateSpace2 = stateSpaceSE3([-250, 250; -250, 250; -250, 250; -pi, pi; inf inf; inf inf; inf inf]);
            obj.stateValidator2 = validatorOccupancyMap3D(obj.stateSpace2,"Map",obj.omap);
            obj.stateValidator2.ValidationDistance = 1;
            
            % Setup Second Planner
            obj.planner2 = plannerRRT(obj.stateSpace2,obj.stateValidator2);
            obj.planner2.MaxConnectionDistance = 1;
            obj.planner2.MaxIterations = 1000;
            obj.planner2.GoalBias = 0.1;
            obj.planner2.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 1);
            
        end
    end
end