classdef auvScenarioHillsSimulation_MultiMode < matlab.System
    %This system is used to visualize and map uuv scenarios
    
    properties(Nontunable)
        RollAccuracy = 0.2
        PitchAccuracy = 0.2
        YawAccuracy = 1
        PositionAccuracy = 1
        VelocityAccuracy = 0.05
    end
    
    properties(Access = private)
        scenario
        lidar
        lidarPlot
        lidarTransform
        ins
        gps
        platform
        scenarioAx
        scenarioFrames
        visual
        visualAx
        visualHandle
        mapping
        mappingAx
        flagtrigger
        omapVisIdx
        OmapVisInterval = 10
        ReplanRequest = 0;
        Reset = [1 1 1 1 1];
        
    end
    
    % Pre-computed constants
    properties(Access = private)
        
    end
    methods(Access = protected)
        
        function [poseUUV, VisualizationStatus,ReplanRequest,Obstacle] = stepImpl(obj,counter, position, orientation, mapEnable, trigger)
            % return point cloud based on input pose
            ReplanRequest = obj.ReplanRequest;
            
            VisualizationStatus = 0;
            Obstacle = [0 0 0 0 0];
            
            i = counter+1;
            %for i=1:1:length(position)
            obj.scenario.advance();
            obj.platform.move([position(i,:), zeros(1,6), eul2quat(orientation(i,:)), zeros(1,3)]);
            obj.scenario.updateSensors();
            
            [isupdated, lidarSampleTime, PtCloud] = obj.lidar.read();
            
            if(~isempty(PtCloud.XLimits))
                Obstacle = [PtCloud.XLimits(1) position(i,:) 0];
                if (PtCloud.XLimits(1)<20 && ReplanRequest == 0)
                    obj.ReplanRequest = 1;
                    ReplanRequest = obj.ReplanRequest;
                    Row  = find(position(:,1)<-16 & position(:,1)>-18 & position(:,2)>19 & position(:,2)< 21);
                    Obstacle = [PtCloud.XLimits(1) position(Row(2),:) 0];
                    obj.Reset = Obstacle;
                    %poseUUV = [position(i,:) 0];
                    %break;                  
%                 elseif (Obstacle ==  [PtCloud.XLimits(1) position(Row(2),:) 0])
%                     VisualizationStatus=1;
                end
                
            end
            if isupdated
                show3D(obj.scenario,"Time",lidarSampleTime,"FastUpdate", true, "Parent",obj.scenarioAx);
                obj.lidarPlot.XData = reshape(PtCloud.Location(:,:,1),[],1);
                obj.lidarPlot.YData = reshape(PtCloud.Location(:,:,2),[],1);
                obj.lidarPlot.ZData = reshape(PtCloud.Location(:,:,3),[],1);
                obj.lidarPlot.CData = reshape(PtCloud.Location(:,:,3),[],1) - min(reshape(PtCloud.Location(:,:,3),[],1));
                
                motion = read(obj.platform);
                if mapEnable
                    tfpoint = pctransform(PtCloud,obj.lidarTransform);
                    points= [reshape(tfpoint.Location(:,:,1),[],1),...
                        reshape(tfpoint.Location(:,:,2),[],1),reshape(tfpoint.Location(:,:,3),[],1)];
                    pointidx = ~any(isnan(points),2);
                    point = points(pointidx,:);
                    point(isnan(point)) = [];
                    pose = [motion(1:3), motion(10:13)];
                    
                    % Read INS and GPS
                    [isupdatedIns, insSampleTime, insPosMeasStruct] = obj.ins.read();
                    [isupdatedGps, gpsSampleTime, gpsPosMeas(i,:), gpsVel, gpsGroundSpeed,gpsCourse] = obj.gps.read();
                    poseIns = [insPosMeasStruct.Position compact(insPosMeasStruct.Orientation)];
                    
                    if ~isempty(point)
                        insertPointCloud(obj.mapping,[poseIns(2) poseIns(1) -poseIns(3) pose(4:7)],point,90)
                        %insertPointCloud(obj.mapping,[pose(1:3) pose(4:7)],point,90)
                        obj.omapVisIdx = obj.omapVisIdx + 1;
                        if mod(obj.omapVisIdx, obj.OmapVisInterval)
                            show(obj.mapping,"Parent",obj.mappingAx)
                            obj.omapVisIdx = 0;
                        end
                    end
                end
                
                drawnow limitrate
            end
            
            
            
            
            
            poseUUV = [motion(1),motion(2),motion(3),0];
            set(obj.visualHandle,'XData',poseUUV(1),'YData',poseUUV(2),'ZData',poseUUV(3))
            
            if(trigger == 1 && obj.flagtrigger == 0 || trigger == 2 && obj.flagtrigger == 1)
                plot3(obj.visualAx,poseUUV(1),poseUUV(2),poseUUV(3),'.g','MarkerSize',20);
                obj.flagtrigger = obj.flagtrigger + 1;
            end
            if (i==length(position))
                VisualizationStatus=1;
            end
            %end
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.refresh();
        end
        
        function resetImpl(~)
        end
        
        function releaseImpl(obj)
            % Release resources, such as file handles
            %save obj.mapping storedMap;
        end
        
        function [out1,out2,out3,out4] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 4];
            out2 = [1];
            out3 = [1];
            out4 = [1 5];
        end
        
        function [out1,out2,out3,out4] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
        end
        
        function [out1,out2,out3,out4] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
        end
        
        function [out1,out2,out3,out4] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", 0.5);
        end
    end
    
    methods(Access = protected, Static)
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
    
    methods (Access = private)
        
        function refresh(obj)
            %             %setup uavScenario
            %             scene = uavScenario("UpdateRate", 2, "ReferenceLocation", [75 -46 0]);
            %             color.Gray = 0.651*ones(1,3);
            %             color.Green = [0.3922 0.8314 0.0745];
            %             color.Red = [1 0 0];
            %
            %             %Add scenario floor
            %             addMesh(scene,"polygon",{[0 250; 250 250; 250 0; 0 0],[-4 0]},color.Gray)
            %             %Add sets of polygons as extruded meshes
            %             for i=0:1:4
            %                 addMesh(scene,"polygon",{[17+i 204-i; 30-i 204-i; 30-i 154+i; 17+i 154+i],[0 60*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[70+i 174-i; 85-i 174-i; 85-i 131+i; 70+i 131+i],[0 85*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[79+i 180-i; 120-i 180-i; 120-i 158+i; 79+i 158+i],[0 70*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[120+i 173-i; 156-i 173-i; 156-i 161+i; 120+i 161+i],[0 20*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[92+i 143-i; 123-i 143-i; 123-i 120+i; 92+i 120+i],[0 101*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[57+i 87-i; 96-i 87-i; 96 49+i; 57+i 49+i],[0 98*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[149+i 118-i; 154-i 118-i; 154-i 80+i; 149+i 80+i],[0 25*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[179+i 212-i; 184-i 212-i; 184-i 166+i; 179+i 166+i],[0 94*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[186+i 124-i; 203-i 124-i; 203-i 91+i; 185+i 91+i],[0 14*i/4]},color.Green)
            %                 addMesh(scene,"polygon",{[180+i 36-i; 185-i 36-i; 185-i 3+i; 180+i 3+i],[0 69*i/4]},color.Green)
            %             end
            %             %Add blackbox
            %             addMesh(scene,"polygon",{[120 90; 130 90; 130 80; 120 80],[0 3]},color.Red)
            %             obj.scenario = scene;
            %
            load('UUVSceneHighRes.mat')
            load ellipsoidmesh.mat
            addMesh(scene,"polygon",{[-16 6; -16 8; -12 8; -12 6],[0 5]},[1 0 0])
            obj.scenario = scene;
            
            %setup uavPlatform
            %load('ellipsoidmesh.mat');
            plat = uavPlatform("UAV", obj.scenario, "ReferenceFrame", "ENU", ...
                "InitialPosition", [0 0 0], "InitialOrientation", eul2quat([0 0 0]));
            %plat.updateMesh("quadrotor", {10}, [1 0 0], eul2tform([0 0 pi]));
            plat.updateMesh("custom", {v,f}, [1 1 0], [0 0 0], [1 0 0 0]);
            obj.platform = plat;
            
            %setup uavLidarPointCloudGenerator
            lidarmodel = uavLidarPointCloudGenerator(...
                "AzimuthResolution", 0.3324099,"AzimuthLimits", [-60 60], "ElevationLimits", [-20 20],...
                "ElevationResolution", 1.25, "MaxRange", 90, ...
                "HasOrganizedOutput", true, "UpdateRate", 2);
            obj.lidar = uavSensor("Lidar", obj.platform, lidarmodel,"MountingLocation", [0,0,-4.5], "MountingAngles",[0 90 0]);
            
            %setup uav ins
            insmodel= insSensor('RollAccuracy',0.2,'PitchAccuracy',0.2,'YawAccuracy',1,'PositionAccuracy',1,'VelocityAccuracy',0.05);
            obj.ins = uavSensor("INS", obj.platform, insmodel,"MountingLocation", [0,0,0], "MountingAngles",[0 0 0]);
            %setup uav gps
            gpsmodel= gpsSensor();
            obj.gps = uavSensor("GPS", obj.platform, gpsmodel,"MountingLocation", [0,0,+4.5], "MountingAngles",[0 0 0]);
            
            if isempty(findall(groot,'Tag','scenario'))
                figure('Tag','scenario');
                [obj.scenarioAx,obj.scenarioFrames] = obj.scenario.show3D();
                %obj.scenarioAx.Tag = 'scenario';
                colormap("jet")
                obj.lidarPlot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333],"Parent",obj.scenarioFrames.UAV.Lidar);
            end
            
            setup(obj.scenario);
            
            %setup occupancy map for mapping
            if(~exist('mappedarea.mat','file'))
                obj.mapping = occupancyMap3D(1);
            else
                load('mappedarea.mat')
                obj.mapping = storedMap;
            end
            if isempty(findall(groot,'Tag','mapping'))
                figure('Tag','mapping');
                obj.mappingAx = show(obj.mapping);
                title(obj.mappingAx, 'Mapped Region')
                obj.mappingAx.Tag = 'mapping';
            else
                obj.mappingAx = findall(groot,'Tag','mapping').Children;
            end
            axis(obj.mappingAx,'equal');
            hold on
            
            %setup visualization map
            mapData = load("UUVSceneLowResMap3.mat", "omap");
            obj.visual = mapData.omap;
            obj.visual.FreeThreshold = obj.visual.OccupiedThreshold;
            inflate(obj.visual, 1);
            
            if isempty(findall(groot,'Tag','visual'))
                figure('Tag','visual');
                obj.visualAx = show(obj.visual);
                title(obj.visualAx, 'Localization Map')
                hold on
                obj.visualAx.Tag = 'visual';
                obj.visualHandle = plot3(obj.visualAx,0,0,0,'.r','MarkerSize',20);
            end
            
            
            theta = pi/2;
            rot = [cos(theta) 0 -sin(theta); ...
                0    1 0         ; ...
                sin(theta) 0  cos(theta);];
            trans = [0, 0, 0];
            obj.lidarTransform = rigid3d(rot,trans);
            
            obj.flagtrigger = 0;
            obj.omapVisIdx = 0;
            
        end
    end
end
