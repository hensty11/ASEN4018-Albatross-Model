function [T_available_N, P_shaft_W, P_electrical_W, ...
    eta_prop, eta_motor, eta_ESC, msgs, RPM_operating, ...
    T_operating_N, feasible] = ElectricPropulsion( ...
    Msn_Alt_ft, Msn_Mach, T_required_N, Config_Row, Propulsion_Input, msgs)
% ELECTRICPROPULSION
% Finds a motor/propeller operating point for one mission segment.
%
% Inputs
%   Msn_Alt_ft       Mission-segment average altitude [ft]
%   Msn_Mach         Mission-segment average Mach number [-]
%   T_required_N     Total thrust required by the aircraft [N]
%   Config_Row       Row identifying the propulsion configuration
%   Propulsion_Input Structure containing motor/propeller data
%   msgs             Existing messages structure (retained for compatibility)
%
% Expected Propulsion_Input fields
%   .MotorData       Cell array or struct array, indexed by Config_Row
%   .PropellerData   Cell array or struct array, indexed by Config_Row
%   .Number          Number of identical motor/propeller units
%   .eta_ESC         Optional ESC efficiency; defaults to 0.95
%
% Expected motor fields
%   Kv_rpmPerV, R_ohm, I0_A, Imax_A, Pmax_W, Vmax_V
%
% Expected propeller fields
%   D_m, J_data, CT_data, CP_data
%
% Model:
%   J = V/(n*D), n = RPM/60
%   T = CT*rho*n^2*D^4
%   P_shaft = CP*rho*n^3*D^5
%   I_motor = torque/Kt + I0, Kt = 60/(2*pi*Kv)
%   V_motor = RPM/Kv + I_motor*R
%   P_electrical = V_motor*I_motor/eta_ESC
%
% Pmax_W is treated as maximum continuous shaft power. If your motor
% datasheet defines this as electrical input power, change that check.
%
% T_available_N is the maximum total thrust found among motor-feasible
% operating points. P_shaft_W, P_electrical_W, eta_prop, eta_motor,
% RPM_operating, and T_operating_N describe the selected operating point:
% the lowest-electrical-power point that meets T_required_N. If no point
% meets the required thrust, the outputs describe the feasible point with
% the greatest thrust and feasible is false.

%% Initialize outputs
T_available_N = 0;
P_shaft_W = NaN;
P_electrical_W = NaN;
eta_prop = NaN;
eta_motor = NaN;
eta_ESC = NaN;
RPM_operating = NaN;
T_operating_N = 0;
feasible = false;

if nargin < 6 || isempty(msgs)
    msgs.warnings = {};
end
if ~isfield(msgs, 'warnings') || isempty(msgs.warnings)
    msgs.warnings = {};
end

if ~isscalar(T_required_N) || ~isfinite(T_required_N) || T_required_N < 0
    error('T_required_N must be a finite, nonnegative scalar in newtons.');
end
if ~isscalar(Msn_Mach) || ~isfinite(Msn_Mach) || Msn_Mach < 0
    error('Msn_Mach must be a finite, nonnegative scalar.');
end

%% Read configuration data
motor = getConfigData(Propulsion_Input.MotorData, Config_Row);
prop = getConfigData(Propulsion_Input.PropellerData, Config_Row);
numUnits = Propulsion_Input.Number(Config_Row);

if isfield(Propulsion_Input, 'eta_ESC')
    eta_ESC = getConfigValue(Propulsion_Input.eta_ESC, Config_Row);
else
    eta_ESC = 0.95; % Typical initial estimate for a brushless ESC
end

requiredMotorFields = {'Kv_rpmPerV','R_ohm','I0_A','Imax_A','Pmax_W','Vmax_V'};
for k = 1:numel(requiredMotorFields)
    if ~isfield(motor, requiredMotorFields{k})
        error('MotorData is missing the field "%s".', requiredMotorFields{k});
    end
end

requiredPropFields = {'D_m','J_data','CT_data','CP_data'};
for k = 1:numel(requiredPropFields)
    if ~isfield(prop, requiredPropFields{k})
        error('PropellerData is missing the field "%s".', requiredPropFields{k});
    end
end

if numUnits < 1 || numUnits ~= floor(numUnits)
    error('Propulsion_Input.Number(Config_Row) must be a positive integer.');
end
if eta_ESC <= 0 || eta_ESC > 1
    error('ESC efficiency must be in the range (0, 1].');
end

Kv = motor.Kv_rpmPerV;  % [RPM/V]
Rmotor = motor.R_ohm;   % [ohm]
I0 = motor.I0_A;        % [A]
Imax = motor.Imax_A;    % [A]
Pmax = motor.Pmax_W;    % [W], assumed continuous shaft-power limit
Vmax = motor.Vmax_V;    % [V]
D = prop.D_m;           % [m]

Jdata = prop.J_data(:);
CTdata = prop.CT_data(:);
CPdata = prop.CP_data(:);

if numel(Jdata) ~= numel(CTdata) || numel(Jdata) ~= numel(CPdata)
    error('J_data, CT_data, and CP_data must have the same number of elements.');
end
if numel(Jdata) < 2 || any(~isfinite(Jdata)) || ...
        any(~isfinite(CTdata)) || any(~isfinite(CPdata))
    error('Propeller coefficient data must contain at least two finite data points.');
end
if D <= 0 || Kv <= 0 || Rmotor < 0 || I0 < 0 || ...
        Imax <= 0 || Pmax <= 0 || Vmax <= 0
    error('Motor and propeller physical inputs must be positive (resistance may be zero).');
end

% Sort coefficient data by advance ratio before interpolation.
[Jdata, sortIndex] = sort(Jdata);
CTdata = CTdata(sortIndex);
CPdata = CPdata(sortIndex);

% Remove duplicate J values so interp1 is well-defined.
[Jdata, uniqueIndex] = unique(Jdata, 'stable');
CTdata = CTdata(uniqueIndex);
CPdata = CPdata(uniqueIndex);

%% Atmospheric conditions and flight speed
% The repository's existing mission-segment functions use altitude in ft
% and atmos(...,'units','US'), which returns density in slug/ft^3 and speed
% of sound in ft/s. Convert both to SI for the propeller equations.
[rho_slug_ft3, a_ft_s] = atmos(Msn_Alt_ft, 'units', 'US');
rho = rho_slug_ft3 * 515.3788; % [kg/m^3]
V = Msn_Mach * a_ft_s * 0.3048; % [m/s]

if ~isfinite(rho) || rho <= 0 || ~isfinite(V)
    error('Atmosphere model returned invalid density or flight speed.');
end

rpmMax = Kv * Vmax; % approximate no-load RPM limit

%% Build candidate propeller operating points
% For forward flight, search the tabulated advance-ratio range. RPM follows
% directly from J = V/(nD), so this searches propeller operating points rather
% than arbitrarily sweeping RPM.
if V > 1e-6
    JminAllowed = max(min(Jdata), 60 * V / (rpmMax * D));
    JmaxAllowed = max(Jdata);

    if JminAllowed >= JmaxAllowed
        msgs.warnings{end+1} = ...
            'No propeller-table advance ratios are compatible with the motor RPM limit.';
        warning('%s', msgs.warnings{end});
        return
    end

    Jcandidate = linspace(JminAllowed, JmaxAllowed, 3000);
    nCandidate = V ./ (Jcandidate * D); % rev/s
    rpmCandidate = 60 * nCandidate;
    CTcandidate = interp1(Jdata, CTdata, Jcandidate, 'linear', NaN);
    CPcandidate = interp1(Jdata, CPdata, Jcandidate, 'linear', NaN);
else
    % Static thrust: J = 0. RPM cannot be calculated from J=V/(nD) at V=0.
    % Use the J=0 coefficients and sweep RPM directly.
    if min(Jdata) > 0 || max(Jdata) < 0
        msgs.warnings{end+1} = ...
            'Static thrust requires propeller coefficient data that includes J = 0.';
        warning('%s', msgs.warnings{end});
        return
    end
    CTstatic = interp1(Jdata, CTdata, 0, 'linear');
    CPstatic = interp1(Jdata, CPdata, 0, 'linear');
    rpmCandidate = linspace(max(1, rpmMax/3000), rpmMax, 3000);
    nCandidate = rpmCandidate / 60;
    CTcandidate = repmat(CTstatic, size(rpmCandidate));
    CPcandidate = repmat(CPstatic, size(rpmCandidate));
end

%% Evaluate propeller, motor, and ESC at each candidate
bestThrust = -Inf;
bestThrustData = struct();
bestPower = Inf;
bestRequiredData = struct();

for k = 1:numel(rpmCandidate)
    rpm = rpmCandidate(k);
    n = nCandidate(k);
    CT = CTcandidate(k);
    CP = CPcandidate(k);

    if ~isfinite(CT) || ~isfinite(CP) || CT <= 0 || CP <= 0 || rpm <= 0
        continue
    end

    % Single-propeller thrust and shaft power [SI units].
    Tprop = CT * rho * n^2 * D^4;
    PshaftProp = CP * rho * n^3 * D^5;

    if ~isfinite(Tprop) || ~isfinite(PshaftProp) || ...
            Tprop < 0 || PshaftProp <= 0
        continue
    end

    % Treat the selected motor/propeller units as identical and equally loaded.
    % The motor drives one propeller; totals are multiplied by numUnits below.
    omega = 2*pi*n; % [rad/s]
    torque = PshaftProp / omega; % [N*m]
    Kt = 60 / (2*pi*Kv); % [N*m/A], Kv in RPM/V
    Imotor = torque / Kt + I0;
    Vmotor = rpm / Kv + Imotor * Rmotor;
    PmotorElectrical = Vmotor * Imotor;
    PescElectrical = PmotorElectrical / eta_ESC;
    etaMotorCandidate = PshaftProp / max(PmotorElectrical, eps);
    etaPropCandidate = (Tprop * V) / PshaftProp;

    % Candidate is only acceptable if continuous operating limits are met.
    motorOK = rpm <= rpmMax && ...
        Imotor <= Imax && Vmotor <= Vmax && ...
        PshaftProp <= Pmax && PmotorElectrical > 0 && ...
        etaMotorCandidate > 0 && etaMotorCandidate <= 1.05;

    if ~motorOK
        continue
    end

    % Keep track of maximum thrust the motor can produce within limits.
    if Tprop > bestThrust
        bestThrust = Tprop;
        bestThrustData.rpm = rpm;
        bestThrustData.Tprop = Tprop;
        bestThrustData.Pshaft = PshaftProp;
        bestThrustData.Pelectrical = PescElectrical;
        bestThrustData.etaProp = etaPropCandidate;
        bestThrustData.etaMotor = etaMotorCandidate;
    end

    % Among points that meet demand, choose the lowest electrical power.
    if Tprop * numUnits >= T_required_N && PescElectrical < bestPower
        bestPower = PescElectrical;
        bestRequiredData.rpm = rpm;
        bestRequiredData.Tprop = Tprop;
        bestRequiredData.Pshaft = PshaftProp;
        bestRequiredData.Pelectrical = PescElectrical;
        bestRequiredData.etaProp = etaPropCandidate;
        bestRequiredData.etaMotor = etaMotorCandidate;
    end
end

%% Return selected point and feasibility
if isfinite(bestThrust)
    T_available_N = bestThrust * numUnits;
end

if isfield(bestRequiredData, 'rpm')
    selected = bestRequiredData;
    feasible = true;
elseif isfield(bestThrustData, 'rpm')
    % No point meets thrust demand: return the highest-thrust feasible point
    % for diagnostic purposes, but mark the segment infeasible.
    selected = bestThrustData;
    feasible = false;
    msgs.warnings{end+1} = sprintf( ...
        ['Required thrust %.2f N exceeds available thrust %.2f N at this ' ...
         'mission condition.'], T_required_N, T_available_N);
    warning('%s', msgs.warnings{end});
else
    msgs.warnings{end+1} = ...
        'No motor-feasible propeller operating point was found.';
    warning('%s', msgs.warnings{end});
    return
end

RPM_operating = selected.rpm;
T_operating_N = selected.Tprop * numUnits;
P_shaft_W = selected.Pshaft * numUnits;
P_electrical_W = selected.Pelectrical * numUnits;
eta_prop = selected.etaProp;
eta_motor = selected.etaMotor;

end


function value = getConfigData(allData, row)
% Supports either a cell array of structs or a struct array.
if iscell(allData)
    value = allData{row};
elseif isstruct(allData)
    value = allData(row);
else
    error('MotorData and PropellerData must be cell arrays or struct arrays.');
end
end


function value = getConfigValue(allValues, row)
% Supports a numeric vector or cell array of scalar values.
if iscell(allValues)
    value = allValues{row};
else
    value = allValues(row);
end
end
