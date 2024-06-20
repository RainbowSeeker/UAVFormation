function cellInfo = Other_Mission_Data(varargin) 
% OTHER_MISSION_DATA returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'Other_Mission_Data_Bus', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', ... 
    '0', {... 
{'timestamp', 1, 'uint32', 'real', 'Sample', 'Fixed', [], [], sprintf('ms'), ''}; ...
{'type', 3, 'uint32', 'real', 'Sample', 'Fixed', [], [], '', sprintf('FormAssemble(1),\nFormDisband(2),\nFormMission(3),\nMission(4),')}; ...
{'valid_items', 3, 'uint8', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'x', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'y', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'z', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'heading', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'ext1', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'ext2', [3 8], 'single', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 
