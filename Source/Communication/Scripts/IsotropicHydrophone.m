classdef (Sealed,StrictDefaults) IsotropicHydrophone < phased.internal.AbstractIsotropicSonarElement
%IsotropicHydrophone Isotropic hydrophone element
%   H = phased.IsotropicHydrophone creates an isotropic hydrophone system
%   object, H. This object models a hydrophone element whose response is
%   one in all directions.
%
%   H = phased.IsotropicHydrophone(Name,Value) creates an isotropic
%   hydrophone object, H, with the specified property Name set to the
%   specified Value. You can specify additional name-value pair arguments
%   in any order as (Name1,Value1,...,NameN,ValueN).
%
%   The 0 degree azimuth and 0 degree elevation direction is considered to
%   be the main response axis of the hydrophone. When placed in a linear or
%   a rectangular array, the main response axis is aligned with the array
%   normal defined by the ArrayAxis property for a linear array or the
%   ArrayNormal property for a rectangular array.
%
%   Step method syntax:
%
%   RESP = step(H,FREQ,ANGLE) returns the hydrophone voltage sensitivity
%   in V/Pa, RESP, given the hydrophone's operating frequency FREQ (in
%   Hz) and the directions specified in ANGLE (in degrees). FREQ is a row
%   vector of length L and ANGLE can be either a row vector of length M or
%   a 2xM matrix. RESP is an MxL matrix whose columns contain the responses
%   of the hydrophone element at angles specified in ANGLE at corresponding
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
%   IsotropicHydrophone methods:
%
%   step                  - Output the response of the hydrophone element
%   release               - Allow property name and input characteristics
%                           changes
%   clone                 - Create an isotropic hydrophone object with 
%                           same property values
%   isLocked              - Locked status (logical)
%   isPolarizationCapable - Indicate if the element is capable of 
%                           simulating polarization
%   directivity           - Compute element directivity
%   beamwidth             - Compute element beamwidth
%   pattern               - Plot element response pattern
%   patternAzimuth        - Plot azimuth pattern
%   patternElevation      - Plot elevation pattern
%
%   IsotropicHydrophone properties:
%
%   FrequencyRange        - Operating frequency range
%   VoltageSensitivity    - Hydrophone sensitivity
%   BackBaffled           - Baffle the back of the element
%
%   % Examples:
%
%   % Example 1:
%   %   Construct an isotropic hydrophone and plot its pattern.  
%   %   Assume the hydrophone operates between 1 and 20 kHz and the
%   %   operating frequency is 10 kHz.
%
%   hydrophone = phased.IsotropicHydrophone('FrequencyRange',[1e3 20e3]);
%   fc = 10e3;
%   pattern(hydrophone,fc);
%
%   % Example 2:
%   %   Find the response of a hydrophone with a -100 dB voltage 
%   %   sensitivity at boresight.
%
%   hydrophone = phased.IsotropicHydrophone('VoltageSensitivity',-100);
%   fc = 20e3; ang = [0;0];
%   resp = hydrophone(fc,ang)
%
%   See also phased, phased.IsotropicProjector,
%   phased.IsoSpeedUnderwaterPaths, phased.ULA, phased.URA,
%   phased.ConformalArray.

%   Copyright 2016-2020 The MathWorks, Inc.

%#ok<*EMCLS>
%#ok<*EMCA>
%#codegen

    properties (Nontunable)
        %VoltageSensitivity Hydrophone voltage sensitivity (dB)
        %   Specify the hydrophone voltage sensitivity in dB relative to 1
        %   V/uPa as a scalar or a row vector. The default value is -120.
        VoltageSensitivity = -120;
    end
    
    methods
        function obj=IsotropicHydrophone(varargin)
            obj=obj@phased.internal.AbstractIsotropicSonarElement(varargin{:});
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj,~,~)
            if length(obj.VoltageSensitivity)>1 
                obj.pFrequencyVector=linspace(obj.FrequencyRange(1),obj.FrequencyRange(2),length(obj.VoltageSensitivity));
                obj.pResponse = db2mag(obj.VoltageSensitivity)*1e6;
            else  
                obj.pFrequencyVector=obj.FrequencyRange;
                obj.pResponse = db2mag(obj.VoltageSensitivity)*1e6*ones(size(obj.pFrequencyVector));
            end
        end
    end

    methods
      function set.VoltageSensitivity(obj,val)
          validateattributes( val, { 'double' }, ...
              {'finite','row'},'','VoltageSensitivity');
          obj.VoltageSensitivity = val;
      end
    end
    
    methods (Hidden)
        function clIso = clonecg(obj)
            clIso = phased.IsotropicHydrophone(...
                'FrequencyRange',obj.FrequencyRange, ...
                'BackBaffled',obj.BackBaffled, ...
                'VoltageSensitivity',obj.VoltageSensitivity);      
        end
    end
end

            
