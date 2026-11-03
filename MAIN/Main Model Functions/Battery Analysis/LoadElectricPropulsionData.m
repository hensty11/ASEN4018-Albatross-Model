function [Propulsion_Input, chem, batteryOptions] = LoadElectricPropulsionData(Propulsion_Input, workbookPath)
%LOADELECTRICPROPULSIONDATA Load motor, propeller, and generic battery inputs.
% Workbook sheets: MotorData, PropellerData, BatteryModelInputs.
% Populate the template with real motor/propeller data before running.

if nargin < 2 || isempty(workbookPath)
    thisFile = mfilename('fullpath');
    mainDir = fileparts(fileparts(fileparts(thisFile))); % .../MAIN
    workbookPath = fullfile(mainDir,'Input Excel Sheets','Electric Propulsion Data', ...
        'ElectricPropulsion_Input_FilledTest.xlsx');
end
if ~isfile(workbookPath)
    error('Electric propulsion workbook not found: %s', workbookPath);
end
motorTable = readtable(workbookPath,'Sheet','MotorData','VariableNamingRule','preserve');
propTable = readtable(workbookPath,'Sheet','PropellerData','VariableNamingRule','preserve');
battCells = readcell(workbookPath,'Sheet','BatteryModelInputs');

nConfig = numel(Propulsion_Input.PropType);
Propulsion_Input.MotorData = cell(nConfig,1);
Propulsion_Input.PropellerData = cell(nConfig,1);
Propulsion_Input.MotorCandidates = cell(nConfig,1);
Propulsion_Input.PropellerCandidates = cell(nConfig,1);
% Optional independent motor-winding comparison data; reference motor remains unchanged.
try
    candidateTable = readtable(workbookPath,'Sheet','MotorCandidates','VariableNamingRule','preserve');
catch
    candidateTable = table();
end
try
    propCandidateTable = readtable(workbookPath,'Sheet','PropellerCandidates','VariableNamingRule','preserve');
catch
    propCandidateTable = table();
end
% Propulsion_Input is a MATLAB table: each added variable must have nConfig rows.
if ~ismember('eta_ESC', Propulsion_Input.Properties.VariableNames)
    Propulsion_Input.eta_ESC = 0.95*ones(nConfig,1);
end
for cfg = 1:nConfig
    Propulsion_Input.MotorData{cfg} = [];
    Propulsion_Input.PropellerData{cfg} = [];
    if ~strcmp(string(Propulsion_Input.PropType{cfg}),"PROP_Electric")
        continue
    end
    mi = find(motorTable.Config_Row == cfg,1,'first');
    pi = find(propTable.Config_Row == cfg,1,'first');
    % The design workbook can contain other electric test configurations.
    % Leave them empty unless explicit matching data is supplied; sizing.m
    % validates the selected Config_Row before using it.
    if isempty(mi) || isempty(pi)
        continue
    end
    motor = struct('Kv_rpmPerV',motorTable.Kv_rpmPerV(mi), ...
        'R_ohm',motorTable.R_ohm(mi),'I0_A',motorTable.I0_A(mi), ...
        'Imax_A',motorTable.Imax_A(mi),'Pmax_W',motorTable.Pmax_W(mi), ...
        'Vmax_V',motorTable.Vmax_V(mi));
    prop = struct('D_m',propTable.D_m(pi), ...
        'J_data',parseNumericList(propTable.J_data(pi)), ...
        'CT_data',parseNumericList(propTable.CT_data(pi)), ...
        'CP_data',parseNumericList(propTable.CP_data(pi)));
    validateMotor(motor,cfg);
    validatePropeller(prop,cfg);
    Propulsion_Input.MotorData{cfg} = motor;
    Propulsion_Input.PropellerData{cfg} = prop;
    variants = motor; variants.Motor_ID = 'Reference motor';
    if ~isempty(candidateTable) && ismember('Config_Row',candidateTable.Properties.VariableNames)
        idxs=find(candidateTable.Config_Row==cfg);
        for jj=idxs(:)'
            m2=struct('Kv_rpmPerV',candidateTable.Kv_rpmPerV(jj), ...
                'R_ohm',candidateTable.R_ohm(jj),'I0_A',candidateTable.I0_A(jj), ...
                'Imax_A',candidateTable.Imax_A(jj),'Pmax_W',candidateTable.Pmax_W(jj), ...
                'Vmax_V',candidateTable.Vmax_V(jj), ...
                'Motor_ID',char(string(candidateTable.Motor_ID(jj))));
            validateMotor(m2,cfg);
            if abs(m2.Kv_rpmPerV-motor.Kv_rpmPerV)>1e-9
                variants(end+1)=m2; %#ok<AGROW>
            end
        end
    end
    Propulsion_Input.MotorCandidates{cfg}=variants;
    propVariants=prop; propVariants.Propeller_ID='Reference APC 16x16';
    if ~isempty(propCandidateTable) && ismember('Config_Row',propCandidateTable.Properties.VariableNames)
        idxs=find(propCandidateTable.Config_Row==cfg);
        for jj=idxs(:)'
            pv=struct('D_m',propCandidateTable.D_m(jj), ...
                'J_data',parseNumericList(propCandidateTable.J_data(jj)), ...
                'CT_data',parseNumericList(propCandidateTable.CT_data(jj)), ...
                'CP_data',parseNumericList(propCandidateTable.CP_data(jj)), ...
                'Propeller_ID',char(string(propCandidateTable.Propeller_ID(jj))));
            validatePropeller(pv,cfg);
            if abs(pv.D_m-prop.D_m)>1e-9 || numel(pv.J_data)~=numel(prop.J_data) || ...
                    any(abs(pv.CT_data(:)-prop.CT_data(:))>1e-8)
                propVariants(end+1)=pv; %#ok<AGROW>
            end
        end
    end
    Propulsion_Input.PropellerCandidates{cfg}=propVariants;
end

% Read battery parameters by name so spreadsheet row order can change.
chem = struct();
for k = 2:size(battCells,1)
    key = string(battCells{k,1});
    val = battCells{k,2};
    if ismissing(key) || strlength(strtrim(key))==0 || isempty(val), continue; end
    switch key
        case {'soc_points','ocv_cell_V'}
            chem.(char(key)) = parseNumericList(val);
        case {'nominalCellV','maxCellV','minCellV','specificEnergy_Whkg', ...
                'energyDensity_WhL','R_cell_ohmAh','usableCapacityFraction', ...
                'minimumFinalSOC','peukertExponent','ratedDischargeRate_C','referenceCellCapacity_Ah'}
            chem.(char(key)) = parseScalar(val);
    end
end
% v9: the supplied 36.8 Ah value describes a REFERENCE PACK/BLOCK, not
% an independently validated cell size. Never round the optimized capacity
% in 36.8 Ah increments: doing so doubled mass on the 1P -> 2P transition.
% Keep the reference for comparison, but enable continuous equivalent capacity
% until a real chemistry cell capacity is supplied and validated.
if isfield(chem,'referenceCellCapacity_Ah')
    chem.referencePackCapacity_Ah = chem.referenceCellCapacity_Ah;
    chem = rmfield(chem,'referenceCellCapacity_Ah');
end
% v9 policy: end the mission at no less than 5%% of modeled charge.
% This replaces the previous workbook reserve (not the cell voltage cutoff).
chem.minimumFinalSOC = 0.05;
batteryOptions = struct();
for k = 2:size(battCells,1)
    key = string(battCells{k,1}); val = battCells{k,2};
    if ismissing(key) || strlength(strtrim(key))==0 || isempty(val), continue; end
    switch key
        case {'P_avionics_W','wiringLossFraction','energyMarginFactor','maxTimeStep_s','Vmin_bus_V','Vmax_bus_V', ...
                'StartupTaxi_duration_s','StartupTaxi_powerFraction','Descent_duration_s','Descent_powerFraction', ...
                'Landing_duration_s','Landing_powerFraction'}
            batteryOptions.(char(key)) = parseScalar(val);
    end
end
% Defaults for phases whose current mission-template rows do not contain duration/speed data.
% These are explicitly approximate and should be replaced with project-specific estimates.
auxNames = {'StartupTaxi_duration_s','StartupTaxi_powerFraction','Descent_duration_s','Descent_powerFraction','Landing_duration_s','Landing_powerFraction'};
auxDefaults = [300 0.05 600 0.05 60 0.05];
for q=1:numel(auxNames)
    if ~isfield(batteryOptions,auxNames{q}), batteryOptions.(auxNames{q})=auxDefaults(q); end
end
Propulsion_Input.ElectricAuxSegmentDefaults = repmat({batteryOptions},nConfig,1);
% Convert spreadsheet options to BatteryRequirements option names.
batteryOptions.P_avionics_W = getOption(batteryOptions,'P_avionics_W',15);
batteryOptions.wiringLossFraction = getOption(batteryOptions,'wiringLossFraction',0.03);
batteryOptions.energyMarginFactor = getOption(batteryOptions,'energyMarginFactor',1.10);
batteryOptions.maxTimeStep_s = getOption(batteryOptions,'maxTimeStep_s',10);
batteryOptions.nSeries = []; % v7: optimized, NOT the reference pack's 12S
% Vmin_bus_V / Vmax_bus_V in the workbook describe a sample 12S pack.
% They are not hard limits for the unconstrained chemistry-based design search.

% Clearly reject the intentionally unfilled motor/propeller template rows.
if any(~isfinite([chem.nominalCellV chem.maxCellV chem.minCellV chem.specificEnergy_Whkg chem.energyDensity_WhL chem.R_cell_ohmAh]))
    error('BatteryModelInputs has missing required numeric values.');
end
if ~isfield(batteryOptions,'Vmin_bus_V') || ~isfield(batteryOptions,'Vmax_bus_V') || ...
        batteryOptions.Vmin_bus_V<=0 || batteryOptions.Vmax_bus_V<=batteryOptions.Vmin_bus_V
    error('BatteryModelInputs requires Vmin_bus_V > 0 and Vmax_bus_V > Vmin_bus_V.');
end
end

function x = parseScalar(v)
if isnumeric(v), x=double(v); else, x=str2double(string(v)); end
if ~isscalar(x) || ~isfinite(x), error('Expected a finite numeric scalar in BatteryModelInputs.'); end
end

function v = getOption(s,field,defaultValue)
if isfield(s,field) && ~isempty(s.(field)) && isfinite(s.(field)), v=s.(field); else, v=defaultValue; end
end

function x = parseNumericList(v)
% Robustly parse numeric vectors imported from mixed-type Excel columns.
% Supports numeric arrays, cells, strings, character vectors, and semicolon /
% comma / whitespace separated lists. Include the bad value in any error.
while iscell(v)
    if isempty(v)
        error('BatteryModelInputs contains an empty numeric-list cell.');
    end
    v = v{1};
end
if isnumeric(v)
    x = double(v(:))';
else
    if ismissingStringValue(v)
        error('BatteryModelInputs contains an empty numeric-list value.');
    end
    s = strtrim(string(v));
    if ~isscalar(s) || ismissing(s) || strlength(s)==0 || contains(s,'REPLACE_WITH')
        error('Replace placeholder/empty numeric-list values in the electric propulsion workbook.');
    end
    % Excel sometimes supplies a cell as a char/string with line breaks.
    s = replace(s, newline, ';');
    s = replace(s, char(13), ';');
    parts = split(s, {';', ',', ' ', char(9)});
    parts = strtrim(parts(:));
    parts(parts=="") = [];
    x = str2double(parts)';
end
if isempty(x) || any(~isfinite(x))
    if isstring(v) && (isempty(v) || any(ismissing(v)))
        shown = '<missing/empty>';
    elseif isempty(v)
        shown = '<empty>';
    else
        shown = char(string(v));
    end
    error('Invalid numeric list in BatteryModelInputs. Value received: "%s". Use numbers separated by semicolons.', shown);
end
end

function tf = ismissingStringValue(v)
tf = isempty(v);
if isstring(v)
    tf = tf || any(ismissing(v)) || all(strlength(strtrim(v))==0);
elseif ischar(v)
    tf = tf || isempty(strtrim(v));
end
end

function validateMotor(m,cfg)
n = [m.Kv_rpmPerV m.R_ohm m.I0_A m.Imax_A m.Pmax_W m.Vmax_V];
if any(~isfinite(n)) || m.Kv_rpmPerV<=0 || m.R_ohm<0 || m.I0_A<0 || ...
        m.Imax_A<=0 || m.Pmax_W<=0 || m.Vmax_V<=0
    error('Invalid or missing motor data for Config_Row %d.',cfg);
end
end

function validatePropeller(p,cfg)
if p.D_m<=0 || numel(p.J_data)<2 || ...
        numel(p.J_data)~=numel(p.CT_data) || numel(p.J_data)~=numel(p.CP_data) || ...
        any(diff(sort(p.J_data))==0) || any(~isfinite([p.J_data p.CT_data p.CP_data]))
    error('Invalid propeller data for Config_Row %d. J, CT, and CP arrays must match and have unique J values.',cfg);
end
end
