function [W_Mat,BMF_Mat,DATA,msgs,mission,Battery_Results] = OptimizeElectricMissionBattery( ...
    MSN_Profile,W0_guess,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs,chem,opts)
% v10: motor winding + battery optimization. Practical 12S trials first;
% if none work, compare/reference higher voltage as a clearly labeled fallback.
% Every motor candidate must have actual KV/R/Io/Imax/Vmax/Pmax data.
% Candidate motor limits are taken from manufacturer saturation ratings; do
% not interpret them as qualified continuous thermal limits.
cfg=Constants{1}; original=Propulsion_Input.MotorData{cfg};
if ismember('MotorCandidates',Propulsion_Input.Properties.VariableNames) && ...
        ~isempty(Propulsion_Input.MotorCandidates{cfg})
    motors=Propulsion_Input.MotorCandidates{cfg};
else
    motors=original;
    motors.Motor_ID='Reference motor';
end
if ismember('PropellerCandidates',Propulsion_Input.Properties.VariableNames) && ...
        ~isempty(Propulsion_Input.PropellerCandidates{cfg})
    props=Propulsion_Input.PropellerCandidates{cfg};
else
    props=Propulsion_Input.PropellerData{cfg};
    props.Propeller_ID='Reference propeller';
end
if ~isfield(motors,'Motor_ID')
    for k=1:numel(motors), motors(k).Motor_ID=sprintf('Candidate %d',k); end
end
% 12S is a practical target, NOT the nominal reference battery as a physical cell.
% Full-charge voltage must satisfy motor Vmax (and eventual ESC Vmax).
referenceNs=12;
bestMass=Inf; good=false;
notes=cell(0,1);
for k=1:numel(motors)
    motor=motors(k);
    if referenceNs*chem.maxCellV>motor.Vmax_V
        notes{end+1,1}=sprintf('%s: 12S full-charge voltage exceeds motor rating',motor.Motor_ID); %#ok<AGROW>
        continue
    end
    for q=1:numel(props)
        [ok,res,note]=evaluateTrial(motor,props(q),referenceNs);
        notes{end+1,1}=sprintf('%s + %s at 12S: %s',motor.Motor_ID,props(q).Propeller_ID,note); %#ok<AGROW>
        if ok
            if res.bat.mass_kg < bestMass
                bestMass=res.bat.mass_kg; winner=res;good=true;
            end
        end
    end
end
% Explicitly do not quietly choose >12S if 12S works. If not feasible,
% search the known/reference winding at higher voltages and report selection.
if ~good
    fprintf('[v10] No feasible 12S motor/propeller candidate. Checking higher-voltage fallback.\n');
    maxNs=floor(original.Vmax_V/chem.maxCellV);
    for ns=13:maxNs
        [ok,res,note]=evaluateTrial(motors(1),props(1),ns);
        notes{end+1,1}=sprintf('Reference winding at %dS: %s',ns,note); %#ok<AGROW>
        if ok && res.bat.mass_kg < bestMass
            bestMass=res.bat.mass_kg;winner=res;good=true;
        end
    end
end
if ~good
    disp('v10 motor/battery trial results:');disp(notes);
    error('v10: no valid motor/propeller and battery candidate meets this mission. Review trial notes.');
end
% Save selected operating point and basic trial summary in battery results.
W_Mat=winner.wm;BMF_Mat=winner.bmf;DATA=winner.data;
msgs=winner.messages;mission=winner.mission;Battery_Results=winner.bat;
Battery_Results.selectedMotorID=winner.motorID;
Battery_Results.selectedPropellerID=winner.propellerID;
Battery_Results.selectedMotorKv_rpmPerV=winner.kv;
Battery_Results.selectedMotorMaxVoltage_V=winner.vmax;
Battery_Results.motorCandidateNotes=notes;
Battery_Results.voltageArchitecture=string(sprintf('%dS',Battery_Results.nSeries));
Battery_Results.isPractical12S=(Battery_Results.nSeries==referenceNs);
if ~Battery_Results.isPractical12S
    warning('v10 selected %dS: NO 12S candidate met all modeled mission constraints. Review the motor and ESC selection.',Battery_Results.nSeries);
end
fprintf('[v10] Selected: %s + %s, %dS, %.1f V nominal, %.2f Ah equivalent, %.2f kg, %.1f Wh mission.\n', ...
    Battery_Results.selectedMotorID,Battery_Results.selectedPropellerID,Battery_Results.nSeries, ...
    Battery_Results.nominalPackVoltage_V,Battery_Results.requiredCapacity_Ah, ...
    Battery_Results.mass_kg,Battery_Results.missionEnergy_Wh);
if ~Battery_Results.isPractical12S
    fprintf('[v10] 12S candidate rejection details:\n');
    for j=1:numel(notes)
        if contains(notes{j},'12S'), fprintf('  %s\n',notes{j}); end
    end
end
fprintf('[v10] Manufacturer motor current entries are saturation, not guaranteed continuous thermal limits.\n');

    function [ok,res,note]=evaluateTrial(motor,prop,ns)
        ok=false;res=struct();note='';
        % Keep all other airframe and mission inputs unchanged.
        propTrial=Propulsion_Input;
        propTrial.MotorData{cfg}=motor;
        propTrial.PropellerData{cfg}=prop;
        % Determine bus ceiling using reserve-SOC OCV and 5% sag allowance.
        floorSOC=max(chem.minimumFinalSOC,0.05);
        reserveOCV=interp1(chem.soc_points,chem.ocv_cell_V,floorSOC,'linear');
        busLimit=0.95*ns*reserveOCV;
        if ~isfinite(busLimit) || busLimit<=0
            note='invalid reserve-SOC cell OCV';return
        end
        try
            [wm,bmf,data,messages,missionTry]=MSN_SEG_Handler( ...
                MSN_Profile,W0_guess,Constants,propTrial,DragPolar_Model,WaveDrag_Data,msgs,busLimit);
            req.Vmax_V=motor.Vmax_V;
            req.Vmin_bus_V=max(missionTry.Vmin_motor_V,ns*chem.minCellV);
            trialOpts=opts;trialOpts.nSeries=ns;
            trialOpts.verbose=false;trialOpts.plot=false;
            bat=BatteryRequirements(missionTry,req,chem,trialOpts);
            if ~bat.passed
                note=char(string(bat.failureReason));return
            end
            if any(bat.terminalVoltage_V < max(missionTry.Vmin_motor_V)-1e-6)
                note='battery voltage below motor voltage demand';return
            end
            ok=true;note=sprintf('PASS: %.2f kg / %.1f Wh',bat.mass_kg,bat.missionEnergy_Wh);
            res=struct('wm',wm,'bmf',bmf,'data',{data},'messages',messages, ...
                'mission',missionTry,'bat',bat,'motorID',motor.Motor_ID, ...
                'kv',motor.Kv_rpmPerV,'vmax',motor.Vmax_V, ...
                'propellerID',prop.Propeller_ID);
        catch ME
            note=ME.message;
        end
    end
end
