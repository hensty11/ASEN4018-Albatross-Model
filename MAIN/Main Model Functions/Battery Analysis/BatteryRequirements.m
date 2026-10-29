function Results = BatteryRequirements(mission, motorReq, chem, options)
% BATTERYREQUIREMENTS Preliminary battery sizing from a mission power profile.
%
% This is a conceptual sizing model, not a specific-cell validation tool.
% It estimates the capacity needed to supply the propulsion + avionics load
% through the full mission while accounting for voltage sag and a basic
% Peukert-style capacity correction.
%
% REQUIRED INPUTS
%   mission.dt_s          N x 1 segment durations [s]
%   mission.P_motor_W     N x 1 battery-side electrical propulsion power [W]
%                         Include motor/ESC losses; exclude avionics/wiring.
%
%   motorReq.Vmin_bus_V   Minimum bus voltage required, scalar or N x 1 [V]
%   motorReq.Vmax_V       Maximum full-charge bus voltage allowed [V], unless
%                         options.nSeries is provided.
%
%   chem.soc_points       SOC values for OCV curve, covering 0 to 1
%   chem.ocv_cell_V       Open-circuit cell voltage at each SOC [V]
%   chem.nominalCellV     Nominal cell voltage [V]
%   chem.maxCellV         Maximum charged cell voltage [V]
%   chem.R_cell_ohmAh     Approximate cell resistance [ohm*Ah]; pack R = Ns*K/Q
%   chem.specificEnergy_Whkg  Assumed pack-level specific energy [Wh/kg]
%   chem.energyDensity_WhL   Assumed pack-level energy density [Wh/L]
%
% OPTIONAL CHEM FIELDS
%   minCellV (default 0), usableCapacityFraction (0.90),
%   minimumFinalSOC (0.05), peukertExponent (1.05),
%   ratedDischargeRate_C (1.0).
%
% OPTIONAL OPTIONS FIELDS
%   P_avionics_W (15), wiringLossFraction (0.03), energyMarginFactor (1.10),
%   maxTimeStep_s (5), relTol (0.005), voltageMargin_V (0), nSeries ([]),
%   continuousWindow_s (30), plot (false), verbose (false).
%
% IMPORTANT INTERFACE CONTRACT
%   mission.P_motor_W is battery-side electrical propulsion power. This
%   function adds avionics load and wiring overhead; do not add them twice.
%
% MAIN OUTPUTS
%   Results.mass_kg, nominalPackEnergy_Wh, requiredCapacity_Ah,
%   requiredCapacity_mAh, nSeries, peakCurrent_A,
%   requiredContinuousCurrent_A, minimumTerminalVoltage_V, finalSOC,
%   plus mission time histories and a table of series-count candidates.

if nargin < 4 || isempty(options), options = struct(); end
if nargin < 3 || isempty(chem), error('Provide generic battery chemistry/reference data in chem.'); end
[opt, chem] = applyDefaults(options, chem);

%% Mission inputs
 dt = mission.dt_s(:);
 Pmotor = mission.P_motor_W(:);
 Nseg = numel(dt);
 if Nseg == 0 || numel(Pmotor) ~= Nseg
     error('mission.dt_s and mission.P_motor_W must be nonempty vectors of equal length.');
 end
 if any(~isfinite(dt)) || any(dt <= 0) || any(~isfinite(Pmotor)) || any(Pmotor < 0)
     error('Mission durations must be positive and propulsion power must be nonnegative.');
 end
 if ~isfield(motorReq,'Vmin_bus_V')
     error('motorReq.Vmin_bus_V is required.');
 end
 Vreq = motorReq.Vmin_bus_V(:);
 if isscalar(Vreq), Vreq = repmat(Vreq,Nseg,1); end
 if numel(Vreq) ~= Nseg || any(~isfinite(Vreq)) || any(Vreq <= 0)
     error('motorReq.Vmin_bus_V must be positive, scalar or one value per segment.');
 end
 if isempty(opt.nSeries) && ~isfield(motorReq,'Vmax_V')
     error('Provide motorReq.Vmax_V or set options.nSeries.');
 end
 if opt.P_avionics_W < 0 || opt.wiringLossFraction < 0 || opt.wiringLossFraction >= 1
     error('P_avionics_W must be >= 0 and wiringLossFraction must be in [0,1).');
 end
 if chem.usableCapacityFraction <= 0 || chem.usableCapacityFraction > 1 || ...
         chem.minimumFinalSOC < 0 || chem.minimumFinalSOC >= 1 || ...
         chem.peukertExponent < 1 || chem.ratedDischargeRate_C <= 0
     error('Check usable capacity, reserve SOC, Peukert exponent, and rated discharge rate.');
 end
 if numel(chem.soc_points) ~= numel(chem.ocv_cell_V) || ...
         any(diff(chem.soc_points(:)) <= 0) || chem.soc_points(1) > 0 || chem.soc_points(end) < 1
     error('chem.soc_points and chem.ocv_cell_V must match and cover SOC 0 to 1.');
 end
 if chem.specificEnergy_Whkg <= 0 || chem.energyDensity_WhL <= 0 || chem.R_cell_ohmAh < 0
     error('Energy densities must be positive and resistance constant nonnegative.');
 end

%% Convert segment profile to smaller time steps
PbatSeg = (Pmotor + opt.P_avionics_W) ./ (1 - opt.wiringLossFraction);
maxdt = opt.maxTimeStep_s(:);
if isscalar(maxdt), maxdt = repmat(maxdt,Nseg,1); end
if numel(maxdt) ~= Nseg || any(~isfinite(maxdt)) || any(maxdt <= 0)
    error('maxTimeStep_s must be positive, scalar or one value per segment.');
end
nSub = ceil(dt ./ maxdt);
S.dt = repelem(dt ./ nSub,nSub);
S.P = repelem(PbatSeg,nSub);
S.seg = repelem((1:Nseg)',nSub);
S.Vmin = repelem(Vreq + opt.voltageMargin_V,nSub);
S.n = numel(S.dt);
missionEnergy = sum(PbatSeg .* dt) / 3600;
VminReq = max(Vreq) + opt.voltageMargin_V;

%% Choose series-count candidates
if ~isempty(opt.nSeries)
    NsCandidates = unique(round(opt.nSeries(:)'));
    NsCandidates = NsCandidates(NsCandidates >= 1);
    if isempty(NsCandidates), error('options.nSeries must contain positive integer(s).'); end
    if isfield(motorReq,'Vmax_V') && any(NsCandidates * chem.maxCellV > motorReq.Vmax_V + 1e-9)
        error('A requested series count exceeds motorReq.Vmax_V at full charge.');
    end
else
    NsMax = floor(motorReq.Vmax_V / chem.maxCellV + 1e-9);
    NsMin = max(1,ceil(VminReq / chem.maxCellV));
    NsCandidates = NsMin:NsMax;
end

if isempty(NsCandidates)
    Results = emptyResults('No series count fits the voltage limits.',missionEnergy,VminReq);
    return
end

%% Find minimum capacity for each series count
nC = numel(NsCandidates);
cap = nan(nC,1); capMin = nan(nC,1); mass = inf(nC,1); vol = nan(nC,1);
feasible = false(nC,1); reasons = cell(nC,1); sims = cell(nC,1);
for j = 1:nC
    [capMin(j), sim, ok, reason] = findCapacity(NsCandidates(j),S,chem,opt);
    reasons{j} = reason;
    if ok
        cap(j) = capMin(j) * opt.energyMarginFactor;
        sim = simulatePack(cap(j),NsCandidates(j),S,chem,opt);
        if sim.passed
            feasible(j) = true;
            sims{j} = sim;
            E = NsCandidates(j) * chem.nominalCellV * cap(j);
            mass(j) = E / chem.specificEnergy_Whkg;
            vol(j) = E / chem.energyDensity_WhL;
        else
            reasons{j} = sim.reason;
        end
    end
end

candidateTable = table(NsCandidates(:),feasible,cap,mass,vol,reasons, ...
    'VariableNames',{'nSeries','feasible','capacity_Ah','mass_kg','volume_L','reason'});
idx = find(feasible);
if isempty(idx)
    Results = emptyResults('No series-count candidate could supply the mission at the required bus voltage.',missionEnergy,VminReq);
    Results.candidates = candidateTable;
    Results.candidateFailureReasons = reasons;
    return
end
[~,k] = min(mass(idx)); bestIdx = idx(k);
Ns = NsCandidates(bestIdx); Q = cap(bestIdx); sim = sims{bestIdx};
[Icont,Irms] = currentMetrics(sim,opt.continuousWindow_s);

%% Stable output fields for the aircraft sizing model
Results.status = 'OK';
Results.passed = true;
Results.failureReason = '';
Results.failureSegment = NaN;
Results.failureTime_s = NaN;
Results.nSeries = Ns;
Results.feasibleNsRange = NsCandidates(feasible);
Results.nominalPackVoltage_V = Ns * chem.nominalCellV;
Results.fullChargeVoltage_V = Ns * chem.maxCellV;
Results.requiredCutoffVoltage_V = max(VminReq,Ns * chem.minCellV);
Results.minimumCapacity_Ah = capMin(bestIdx);
Results.requiredCapacity_Ah = Q;
Results.requiredCapacity_mAh = 1000*Q;
Results.nominalPackEnergy_Wh = Results.nominalPackVoltage_V * Q;
Results.mass_kg = mass(bestIdx);
Results.volume_L = vol(bestIdx);
Results.peakCurrent_A = max(sim.current_A);
Results.peakC = Results.peakCurrent_A / Q;
Results.rmsCurrent_A = Irms;
Results.requiredContinuousCurrent_A = Icont;
Results.continuousWindow_s = min(opt.continuousWindow_s,sum(sim.dt));
Results.minimumTerminalVoltage_V = min(sim.terminalVoltage_V);
Results.peakPower_W = max(sim.P_W);
Results.finalSOC = sim.soc(end);
Results.missionEnergy_Wh = missionEnergy;
Results.energyFromCells_Wh = sim.cumCellEnergy_Wh(end);
Results.energyDelivered_Wh = sim.cumDeliveredEnergy_Wh(end);
Results.resistiveLoss_Wh = sim.cumLossEnergy_Wh(end);
Results.packResistance_ohm = sim.R_ohm;
Results.candidates = candidateTable;
Results.time_s = sim.time_s;
Results.stepTime_s = sim.time_s(2:end);
Results.SOC = sim.soc;
Results.current_A = sim.current_A;
Results.terminalVoltage_V = sim.terminalVoltage_V;
Results.openCircuitVoltage_V = sim.ocv_V;
Results.batteryPower_W = sim.P_W;
Results.energyDeliveredHistory_Wh = sim.cumDeliveredEnergy_Wh;
Results.energyFromCellsHistory_Wh = sim.cumCellEnergy_Wh;
Results.resistiveLossHistory_Wh = sim.cumLossEnergy_Wh;
if opt.verbose, printSummary(Results); end
if opt.plot, plotSim(Results,VminReq); end
end

function [Qmin,sim,ok,reason] = findCapacity(Ns,S,chem,opt)
% Search for the minimum capacity that passes the mission voltage/SOC checks.
Vnom = Ns*chem.nominalCellV;
E = sum(S.P.*S.dt)/3600;
Q = max(E/(Vnom*chem.usableCapacityFraction*(1-chem.minimumFinalSOC)),1e-4);
sim = simulatePack(Q,Ns,S,chem,opt);
Qlo = Q; Qhi = Q;
if sim.passed
    % Reduce capacity until an infeasible lower bound is found.
    Qhi = Q;
    bracketFound = false;
    for it = 1:80
        Qtry = Qhi/1.5;
        simTry = simulatePack(Qtry,Ns,S,chem,opt);
        if simTry.passed
            Qhi = Qtry;
        else
            Qlo = Qtry;
            bracketFound = true;
            break
        end
    end
    if ~bracketFound
        % Extremely low loads may remain feasible at the search floor.
        Qlo = 0;
    end
else
    Qlo = Q;
    found = false;
    for it = 1:100
        Qhi = Qlo*1.5;
        sim = simulatePack(Qhi,Ns,S,chem,opt);
        if sim.passed, found = true; break; end
        Qlo = Qhi;
    end
    if ~found
        Qmin = NaN; ok = false; reason = sim.reason; return
    end
end
% Qlo is intended to be infeasible and Qhi feasible.
for it = 1:100
    if (Qhi-Qlo)/Qhi <= opt.relTol, break; end
    Qmid = (Qlo+Qhi)/2;
    smid = simulatePack(Qmid,Ns,S,chem,opt);
    if smid.passed, Qhi = Qmid; else, Qlo = Qmid; end
end
Qmin = Qhi;
sim = simulatePack(Qmin,Ns,S,chem,opt);
ok = sim.passed;
reason = sim.reason;
end

function sim = simulatePack(Q,Ns,S,chem,opt)
% Constant-power battery model with OCV(SOC), pack resistance and Peukert-style use.
F = griddedInterpolant(chem.soc_points(:),Ns*chem.ocv_cell_V(:),'linear','nearest');
R = Ns*chem.R_cell_ohmAh/Q;
Iref = Q*chem.ratedDischargeRate_C;
usableAh = Q*chem.usableCapacityFraction;
n = S.n;
soc = nan(n+1,1); soc(1)=1;
I = nan(n,1); V=nan(n,1); OCV=nan(n,1); P= S.P(:);
reason=''; failStep=NaN; nDone=n;
for k=1:n
    s = min(max(soc(k),0),1);
    ocv = F(s); OCV(k)=ocv;
    disc = ocv^2-4*R*P(k);
    if disc < 0 || ocv <= 0
        reason = 'Required power exceeds the pack model voltage/current capability.';
        failStep=k; nDone=k; break
    end
    cur = 2*P(k)/(ocv+sqrt(disc));
    volt = ocv-cur*R;
    I(k)=cur; V(k)=volt;
    ratio = max(cur/max(Iref,eps),1e-12);
    factor = ratio^(chem.peukertExponent-1);
    if opt.clampPeukertBonus, factor=max(factor,1); end
    deltaAh = cur*factor*S.dt(k)/3600;
    soc(k+1)=soc(k)-deltaAh/usableAh;
    if volt < max(S.Vmin(k),Ns*chem.minCellV)
        reason = 'Loaded voltage falls below the required bus voltage.';
    elseif soc(k+1) < chem.minimumFinalSOC
        reason = 'Estimated capacity is insufficient to complete the mission with the SOC reserve.';
    end
    if ~isempty(reason), failStep=k; nDone=k; break; end
end
if isnan(soc(nDone+1)), soc(nDone+1)=soc(nDone); end
I=I(1:nDone); V=V(1:nDone); OCV=OCV(1:nDone); P=P(1:nDone); dts=S.dt(1:nDone);
sim.passed=isempty(reason); sim.reason=reason; sim.failStep=failStep;
sim.failSegment=NaN; sim.failTime_s=NaN;
if ~isnan(failStep)
    sim.failSegment=S.seg(failStep); sim.failTime_s=sum(S.dt(1:failStep));
end
sim.R_ohm=R; sim.dt=dts; sim.time_s=[0;cumsum(dts)]; sim.soc=soc(1:nDone+1);
sim.current_A=I; sim.terminalVoltage_V=V; sim.ocv_V=OCV; sim.P_W=P;
sim.cumDeliveredEnergy_Wh=[0;cumsum(V.*I.*dts)/3600];
sim.cumCellEnergy_Wh=[0;cumsum(OCV.*I.*dts)/3600];
sim.cumLossEnergy_Wh=[0;cumsum(I.^2*R.*dts)/3600];
end

function [Iwindow,Irms] = currentMetrics(sim,window_s)
I=sim.current_A; dt=sim.dt; t=[0;cumsum(dt)]; Qah=[0;cumsum(I.*dt)]; total=t(end);
Irms=sqrt(sum(I.^2.*dt)/max(total,eps));
W=min(window_s,total);
if W<=0, Iwindow=max(I); return; end
starts=unique([0; max(0,t(t<=total-W)); max(0,t-W)]);
starts=starts(starts<=total-W+1e-9);
ends=starts+W;
avg=(interp1(t,Qah,ends,'linear')-interp1(t,Qah,starts,'linear'))/W;
Iwindow=max(avg);
end

function [opt,chem] = applyDefaults(opt,chem)
d.P_avionics_W=15; d.wiringLossFraction=0.03; d.energyMarginFactor=1.10;
d.maxTimeStep_s=5; d.relTol=5e-3; d.voltageMargin_V=0; d.nSeries=[];
d.continuousWindow_s=30; d.clampPeukertBonus=true; d.verbose=false; d.plot=false;
fn=fieldnames(d); for i=1:numel(fn), if ~isfield(opt,fn{i}), opt.(fn{i})=d.(fn{i}); end, end
c.minCellV=0; c.usableCapacityFraction=0.90; c.minimumFinalSOC=0.05;
c.peukertExponent=1.05; c.ratedDischargeRate_C=1.0;
fn=fieldnames(c); for i=1:numel(fn), if ~isfield(chem,fn{i}), chem.(fn{i})=c.(fn{i}); end, end
if ~isscalar(opt.energyMarginFactor) || opt.energyMarginFactor < 1
    error('energyMarginFactor must be a scalar >= 1.');
end
end

function R = emptyResults(reason,E,Vmin)
R.status='INFEASIBLE'; R.passed=false; R.failureReason=reason;
R.failureSegment=NaN; R.failureTime_s=NaN; R.nSeries=NaN;
R.requiredCutoffVoltage_V=Vmin; R.requiredCapacity_Ah=NaN;
R.requiredCapacity_mAh=NaN; R.minimumCapacity_Ah=NaN;
R.nominalPackEnergy_Wh=NaN; R.mass_kg=Inf; R.volume_L=NaN;
R.missionEnergy_Wh=E;
end

function printSummary(R)
fprintf('\n--- PRELIMINARY BATTERY SIZING ---\n');
fprintf('Series count:              %dS (nominal %.2f V, full %.2f V)\n',R.nSeries,R.nominalPackVoltage_V,R.fullChargeVoltage_V);
fprintf('Capacity:                  %.2f Ah (%.0f mAh)\n',R.requiredCapacity_Ah,R.requiredCapacity_mAh);
fprintf('Nominal energy:             %.1f Wh\n',R.nominalPackEnergy_Wh);
fprintf('Estimated mass / volume:    %.2f kg / %.2f L\n',R.mass_kg,R.volume_L);
fprintf('Peak / window current:      %.1f A / %.1f A\n',R.peakCurrent_A,R.requiredContinuousCurrent_A);
fprintf('Minimum loaded voltage:     %.2f V\n',R.minimumTerminalVoltage_V);
fprintf('Final SOC:                  %.1f %%\n',100*R.finalSOC);
fprintf('Mission load / cell energy: %.1f Wh / %.1f Wh\n',R.missionEnergy_Wh,R.energyFromCells_Wh);
end

function plotSim(R,VminReq)
figure('Name','Preliminary Battery Sizing'); tiledlayout(3,1);
nexttile; plot(R.stepTime_s,R.terminalVoltage_V,'LineWidth',1.3); hold on; yline(VminReq,'--','Required minimum'); ylabel('Voltage [V]'); grid on;
nexttile; plot(R.stepTime_s,R.current_A,'LineWidth',1.3); ylabel('Current [A]'); grid on;
nexttile; plot(R.time_s,100*R.SOC,'LineWidth',1.3); xlabel('Mission time [s]'); ylabel('SOC [%]'); grid on;
end
