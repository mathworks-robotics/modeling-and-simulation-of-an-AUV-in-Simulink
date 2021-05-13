classdef (Sealed,StrictDefaults) IsotropicProjector < phased.internal.AbstractIsotropicSonarElement
%IsotropicProjector Isotropic projector element
%   H = phased.IsotropicProjector creates an isotropic projector system
%   object, H. This object models a projector element whose response is one
%   in all directions.
%
%   H = phased.IsotropicProjector(Name,Value) creates an isotropic
%   projector object, H, with the specified property Name set to the
%   specified Value. You can specify additional name-value pair arguments
%   in any order as (Name1,Value1,...,NameN,ValueN).
%
%   The 0 degree azimuth and 0 degree elevation is considered to be the
%   main response axis of the projector. When placed in a linear or a
%   rectangular array, the main response axis is aligned with the array
%   normal defined by the ArrayAxis property for a linear array or the
%   ArrayNormal property for a rectangular array.
%
%   Step method syntax:
%
%   RESP = step(H,FREQ,ANGLE) returns the projector voltage response in
%   Pa/V, RESP, given the projector's operating frequency FREQ (in Hz) and
%   the directions specified in ANGLE (in degrees). FREQ is a row vector of
%   length L and ANGLE can be either a row vector of length M or a 2xM
%   matrix. RESP is an MxL matrix whose columns contain the responses of
%   the projector element at angles specified in ANGLE at corresponding
%   frequencies specified in FREQ.
%
%   When ANGLE is a 2xM matrix, each column of the matrix specifies the
%   direction in the space in [azimuth; elevation] form. The azimuth angle
%   should be between [-180 180] degrees and the elevation angle should be
%   between [-90 90] degrees. If ANGLE is a length M row vector, each
%   element specifies a direction's azimuth angle and the corresponding
%   elevation angle is assumed to be 0.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(H, x) and y = H(x) are
%   equivalent.
%
%   IsotropicProjector methods:
%
%   step                  - Output the response of the projector element
%   release               - Allow property name and input characteristics
%                           changes
%   clone                 - Create an isotropic projector object with same 
%                           property values
%   isLocked              - Locked status (logical)
%   isPolarizationCapable - Indicate if the element is capable of 
%                           simulating polarization
%   directivity           - Compute element directivity
%   beamwidth             - Compute element beamwidth
%   pattern               - Plot element response pattern
%   patternAzimuth        - Plot azimuth pattern
%   patternElevation      - Plot elevation pattern
%
%   IsotropicProjector properties:
%
%   FrequencyRange        - Operating frequency range
%   VoltageResponse       - Projector voltage response
%   BackBaffled           - Baffle the back of the element
%
%   % Examples:
%
%   % Example 1:
%   %   Find the response of a projector with a 100 dB voltage 
%   %   response at boresight.
%
%   projector = phased.IsotropicProjector('VoltageResponse',100);
%   fc = 20e3; ang = [0;0];
%   resp = projector(fc,ang)
%
%   % Example 2:
%   %   Construct an isotropic projector and plot its azimuth pattern at 
%   %   10 degrees of elevation. Assume the projector operates between 1 
%   %   and 30 kHz and the operating frequency is 20 kHz.
%
%   projector = phased.IsotropicProjector('FrequencyRange',[1e3 30e3]);
%   fc = 20e3;
%   patternAzimuth(projector,fc,10);
%
%   See also phased, phased.IsotropicHydrophone,
%   phased.IsoSpeedUnderwaterPaths, phased.ULA, phased.URA,
%   phased.ConformalArray.

%   Copyright 2016-2020 The MathWorks, Inc.

%#ok<*EMCLS>
%#ok<*EMCA>
%#codegen
    
    properties (Nontunable)
        %VoltageResponse    Projector voltage response (dB)
        %   Specify the projector voltage response in dB relative to 1
        %   uPa/V as a scalar or a row vector. The default value is 120.
        VoltageResponse = 120;
    end
    
    methods
        function obj=IsotropicProjector(varargin)
           obj=obj@phased.internal.AbstractIsotropicSonarElement(varargin{:});
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,~,~)
            if length(obj.VoltageResponse)>1 
                obj.pFrequencyVector=linspace(obj.FrequencyRange(1),obj.FrequencyRange(2),length(obj.VoltageResponse));
                obj.pResponse = db2mag(obj.VoltageResponse)*1e-6;
            else 
                obj.pFrequencyVector=obj.FrequencyRange;
                obj.pResponse = db2mag(obj.VoltageResponse)*1e-6*ones(size(obj.pFrequencyVector));
            end
        end
    end
    
    methods
      function set.VoltageResponse(obj,val)
          validateattributes( val, { 'double'}, ...
              {'finite','row'},'','VoltageResponse');
          obj.VoltageResponse = val;
      end
    end
    
    methods (Hidden)
        function clIso = clonecg(obj)
            clIso = phased.IsotropicProjector(...
                'FrequencyRange',obj.FrequencyRange, ...
                'BackBaffled',obj.BackBaffled, ...
                'VoltageResponse',obj.VoltageResponse);      
        end
    end
end

            
