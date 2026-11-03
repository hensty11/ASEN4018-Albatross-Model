function [W0,FinalWeightData,IterationData,FinalSegmentData,outputTable,msgs]=sizing(MSN_Profile,Config_Row,W0_guess,Design_Input,Propulsion_Input,DragPolar_Model,WaveDrag_Data,W_crew,W_pay_fixed,W_pay_drop,msgs)
    %% Aircraft Design Mission Performance Sizing Analysis
    % ASEN 4138
    % Author: John Mah, Maggie Wussow, Jonathan Morris

    % Description:  This function estimates the fuel burn utilized through a
    % user defined mission profile through multiple iterations to converge on a
    % estimate for aircraft total weight.  The user must first model the aircraft
    % aerodynamically via the aero_analysis.m code or via
    % Open VSP and provide an initial "guess" at the total aircraft weight to
    % begin the iterative process. The mission profile should be defined in
    % an excel sheet (e.g. Mission_Profile_Template.xlsx). The initial 
    % guess weight is updated after each iteration with the average value
    % between the guess weight and the calculated weight at the end of the 
    % mission analysis.  Script will stop iterations once the aircraft total 
    % weight calcluated converges with the initial guess of the
    % aircraft total weight within a user defined percentage.

    %% Current Version:  AY24.??
    % Date Last Change: 23 Nov 24
    % Changes in Current Version: Refactored as a function from main
    % script. Now the code expects a mission profile defined by an excel
    % sheet. This sheet should be read in with Read_MSN_Profile.m which
    % returns a MSN_Profile structure. This profile is then interpreted by
    % the MSN_SEG_Handler.m function which calls the appropriate MSN_SEG
    % funtion and returns weight/battery mass fraction data.
    %
    % Functions & files required to execute this function
    %4138_Design Input File_V24-00.xlsx
    %aero_analyis.m (and its outputs)
    %atmos.m
    %DragPolar.m
    %DragPolar_Function.m
    %InducedDrag.m
    %LD.m
    %ParasiteDrag.m
    %Propulsion.m
    %WaveDrag.m
    %WingGeo.m
    %WingLiftDrag.m
    %MSN_SEG_TO.m
    %MSN_SEG_Climb.m
    %MSN_SEG_Cruise.m
    %MSN_SEG_Loiter.m
    %MSN_SEG_Handler.m



    %% Instructions
    % During initial conceptual evaluation, the ASEN4138_Aircraft_Design_Aero_Model_Main.m script
    % should be executed prior to utilization of this code to create the drag polar and propulsion
    % properties of the aircraft configuration concept required to run a
    % mission performance sizing. However, after refining and modeling the
    % concept in OPEN VSP, you should use the sizing.vsp script with
    % outputs from OPEN VSP.

    %%%%%%%%%%%%%%%%%%TWEAKABLES%%%%%%%%%%%%%%%%%%%%%%
    %These are factors that may need to be editted for certain designs. 
    Converge = .005; %Sets the percent difference between W0_guess and W0_calc to determine solution has converged
    Kvs = 1; %From Raymer Table 6.1 based on variable sweep (1.04) or fixed sweep (1.0)
    Composite_Factor=1; %Using the composite homebuilt model it was found that multiplying the empty weight fraction by 0.8-0.9 lined up more closely with serial produced composite aircraft. The Raymer text reccomends a similar approach.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%Initialization
    %Clear Mission Analyis Variables
    clear SizingStruct Msn_Sizing_Table MsnStruct
    %load(MSN_structure);

    %Start Mission Sizing Iterations
    i = 1; %Iteration number
    Diff_W0 = 1; %Set initial value in total weight difference to 100% or 1

    %% AIRCRAFT WEIGHT AND GEOMETRY INPUTS
    %Aircraft Geometry Constants (either from ASEN4138_Aircaft_Design_Aero_Model.m or
    %manually inputted via OPEN VSP)
    AR = Design_Input.AR_w(Config_Row);
    Sref = Design_Input.Sref_w(Config_Row);
    M_max = Design_Input.M_max(Config_Row);
    g = 32.2; %Accel of gravity (ft/s^2);
    ACType=Design_Input.ACType(Config_Row);


    %% PROPULSION INPUTS
    %Type of propulsion used in design. Only used to output final T/W or
    %P/W for specific configuration evaluated.  Fuel burn done in each
    %mission segment function.
    PropType = Propulsion_Input.PropType(Config_Row);
    if strcmp(string(PropType),"PROP_Electric")
        [Propulsion_Input, Battery_Chemistry, Battery_Options] = LoadElectricPropulsionData(Propulsion_Input);
        if isempty(Propulsion_Input.MotorData{Config_Row}) || isempty(Propulsion_Input.PropellerData{Config_Row})
            error(['No motor/propeller data is mapped to the selected electric configuration row %d. ' ...
                'Set Config_Row in MotorData and PropellerData sheets of ElectricPropulsion_Input_FilledTest.xlsx to %d.'], ...
                Config_Row, Config_Row);
        end
    end %Determine if jet or prop propulsion
    %Jet Engine Values (if propellor propulsion, set all to zero)
    TA_mil_sl = Propulsion_Input.Number(Config_Row)*Propulsion_Input.TA_mil_sl(Config_Row); %Uninstalled Mil power total thrust at sea level (all engines)
    TA_AB_sl = Propulsion_Input.Number(Config_Row)*Propulsion_Input.TA_AB_sl(Config_Row); %Uninstalled AB power total thrust at sea level (all engines)
    
    %Prop Engine Values (if jet propulsion, set all to zero)
    PA_shp_sl = Propulsion_Input.Number(Config_Row)*Propulsion_Input.PA_shp_sl(Config_Row); %shaft horsepower from engine at sea level (hp) total (all engines)
    NumEng=Propulsion_Input.Number(Config_Row);
    if strcmp(string(PropType),"PROP_Electric")
        % Use the detailed motor continuous shaft-power rating for the statistical empty-weight model.
        PA_shp_sl = Propulsion_Input.Number(Config_Row)*Propulsion_Input.MotorData{Config_Row}.Pmax_W/745.699872; % hp
    end


    %% PACKAGE CONSTANTS AND ATMOSPHERE
    Constants = {Config_Row,AR,Sref,M_max,g,PropType}; %Package for use in functions

    %Set sea level std atmosphere values
    [~,a_sl,~,~,~,~] = atmos(0,'units','US'); %sea level std atmosphere properties

    %% MISSION ANALYSIS ITERATIVE SIZING
    %Set Convergence Criteria

    [a,c1,c2,c3,c4,c5,msgs]= WeightModel(PropType,ACType,msgs);

    MaxWeightIterations = 200;
    % v5: diagnostic history is kept separately from legacy IterationData.
    % Columns: iteration, guess, empty, battery, calculated, relative error,
    % mission energy, peak battery power, series cells, capacity Ah, final SOC.
    electricDiagnosticHistory = nan(MaxWeightIterations,11);
    while Diff_W0 >= Converge && i <= MaxWeightIterations

        %Calculate Aircraft Parameters & Empty Weight based on Total Weight
        %(W0) and statistical model
        disp("Iteration "+ num2str(i)+ ": Gross Takeoff Weight Guess: "+ W0_guess)

        if strcmp(PropType,'PROP_Fuel')||strcmp(PropType,'PROP_Electric')||strcmp(PropType,'PROP_Turbocharged')||strcmp(PropType,'PROP_Turboprop')
            Power_Weight_Ratio =PA_shp_sl/W0_guess; %in hp/lb shaft hp at sea level/Wo
            V_max = (M_max*a_sl)*(3600/6076); %Max velocity in terms of knots (must convert ft/s to knots using 6076 ft per nautical mile and 3600 sec per hr).
            WingLoading = W0_guess/Sref;

            %Statistical empty weight fraction model for prop aircaft
            We_W0 = (a*(W0_guess)^c1*(AR)^c2*(Power_Weight_Ratio)^c3*(WingLoading)^c4*(V_max)^c5)*Kvs;
            We_W0=We_W0*Composite_Factor;

            We = We_W0*W0_guess; %Empty weight of aircraft (lb)
        else
            Thrust_Weight_Ratio_mil =TA_mil_sl/W0_guess; %Uninstalled thrust at sea level / Wo
            Thrust_Weight_Ratio_AB = TA_AB_sl/W0_guess; %Uninstalled thrust at sea level / Wo
            Thrust_Weight_Ratio = max(Thrust_Weight_Ratio_AB,Thrust_Weight_Ratio_mil); %Size based on max value of T/W
            WingLoading = W0_guess/Sref;
            
            %Statistical empty weight fraction model for jet aircaft
            We_W0 = (a*(W0_guess)^c1*(AR)^c2*(Thrust_Weight_Ratio)^c3*(WingLoading)^c4*(M_max)^c5)*Kvs;
            We_W0=We_W0*Composite_Factor;

            We = We_W0*W0_guess; %Empty weight of aircraft (lb)
        end
        
        %MSN_SEG_Handler replaces past versions of the code where the users
        %editted calls to MSN_SEG functions manually. MSN_SEG_Handler takes
        %in the mission profile and calls different MSN_SEG functions based
        %on user inputs. Returns weight/battery mass fraction and debugging
        %data for each segment and each convergence iteration.
        try
            if strcmp(string(PropType),"PROP_Electric")
                [W_Mat,BMF_Mat,DATA,msgs,mission,Battery_Results]=OptimizeElectricMissionBattery( ...
                    MSN_Profile,W0_guess,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs,Battery_Chemistry,Battery_Options);
            else
                [W_Mat,BMF_Mat,DATA,msgs,mission]=MSN_SEG_Handler( ...
                    MSN_Profile,W0_guess,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
            end
        catch missionError
            if strcmp(string(PropType),"PROP_Electric")
                fprintf(2,['v5 mission failure: iteration %d, W0 guess %.3f lb, ' ...
                    'empty weight %.3f lb (fraction %.4f).\n'],i,W0_guess,We,We_W0);
                fprintf(2,'Inspect climb thrust/propeller operating point before changing geometry.\n');
            end
            rethrow(missionError);
        end
        config=Constants{1};
        PropType=Propulsion_Input.PropType(config);
        if strcmp(string(PropType),"PROP_Electric")
            % Propulsion model has supplied battery-side electrical power for every mission phase.
            % v7: Battery_Results jointly optimized with motor/propeller voltage above.
            v6PrintMissionBudget(mission,W0_guess,i,Battery_Options);
            if ~isfield(Battery_Results,'passed') || ~Battery_Results.passed || ~isfinite(Battery_Results.mass_kg)
                reason='BatteryRequirements did not find a feasible pack.';
                if isfield(Battery_Results,'failureReason') && ~isempty(Battery_Results.failureReason), reason=Battery_Results.failureReason; end
                error('Electric battery sizing failed at aircraft weight %.2f lb: %s',W0_guess,reason);
            end
            % v6: expose pack capacity, voltage sag and candidate constraints.
            v6PrintBatteryBudget(Battery_Results, Battery_Chemistry, Battery_Options);
            BatteryWeight_lb = Battery_Results.mass_kg*2.2046226218;
            % Battery is a separately sized mass; it is not a mission burn fraction.
            % Battery is accounted for explicitly. This assumes the empirical
            % empty-weight estimate EXCLUDES battery mass. We cannot safely
            % subtract an arbitrary battery fraction without calibrating the
            % statistical regression to the original aircraft weight budget.
            W0_calc = W_crew + W_pay_fixed + W_pay_drop + We + BatteryWeight_lb;
            Diff_W0 = abs(W0_guess-W0_calc)/max(W0_guess,eps);
            IterationData(i,:)=[i,W0_calc,We,BatteryWeight_lb]; %#ok<AGROW>
            electricDiagnosticHistory(i,:)=[i,W0_guess,We,BatteryWeight_lb, ...
                W0_calc,Diff_W0,Battery_Results.missionEnergy_Wh, ...
                Battery_Results.peakPower_W,Battery_Results.nSeries, ...
                Battery_Results.requiredCapacity_Ah,Battery_Results.finalSOC];
            fprintf(['v5 sizing: We=%.2f lb (%.1f%%), battery=%.2f lb, ' ...
                'Wcalc=%.2f lb, relErr=%.3f, mission=%.1f Wh, ' ...
                'peak=%.0f W, pack=%dS %.2f Ah, finalSOC=%.3f\n'], ...
                We,100*We_W0,BatteryWeight_lb,W0_calc,Diff_W0, ...
                Battery_Results.missionEnergy_Wh,Battery_Results.peakPower_W, ...
                Battery_Results.nSeries,Battery_Results.requiredCapacity_Ah, ...
                Battery_Results.finalSOC);
            if We_W0 >= 1
                warning('v5: empirical empty weight fraction %.3f >= 1; check applicability of weight regression.',We_W0);
            end
            i = i + 1;
            W0_guess = (W0_guess+W0_calc)/2;
        else

            W_f_total_used=sum(W_Mat(:,3)); %Total amount of fuel burned for mission
            W_f_total_refuel=sum(W_Mat(:,5)); %Total amount of air refuel added in mission
            W_f_req_internal=W_f_total_used-W_f_total_refuel; %Total amount of fuel that must be carried internally
            W0_calc = W_crew + W_pay_fixed + W_pay_drop + W_f_req_internal + We; %Calcuated aircraft total weight based on mission

            %% Calc Difference Between W0_guess and W0_calc
            Diff_W0 = abs(W0_guess-W0_calc)/W0_guess;
            IterationData(i,:)=[i,W0_calc,We,W_f_total_used]; %Save iteration data

            %% Next Iteration Update
            i = i + 1;
            W0_guess = (W0_guess+W0_calc)/2;


        end

    end

    if Diff_W0 >= Converge
        error('Aircraft weight sizing did not converge within %d iterations. Check propulsion feasibility, empty-weight fraction, and mission inputs.',MaxWeightIterations);
    end

    if strcmp(PropType,"PROP_Electric")
        W0=W0_calc;
        FinalWeightData=table(string(MSN_Profile.legtype(:)),BMF_Mat(:),'VariableNames',{'Leg Type','Mission Energy Fraction'});

        IterationData=array2table(IterationData);
        IterationData.Properties.VariableNames=["Iteration Number","W0 [lb]","Empty Weight [lb]","Battery Weight [lb]"];
        % Battery results are appended after the segment-level performance tables.
        FinalSegmentData=DATA;
        FinalSegmentData{end+1}=Battery_Results;
        diagNames={'Iteration','W0_guess_lb','EmptyWeight_lb','BatteryWeight_lb', ...
            'W0_calculated_lb','RelativeError','MissionEnergy_Wh', ...
            'PeakBatteryPower_W','SeriesCount','Capacity_Ah','FinalSOC'};
        FinalSegmentData{end+1}=array2table(electricDiagnosticHistory(1:i-1,:), ...
            'VariableNames',diagNames);

    else
        W0=W0_calc;
        FinalWeightData=[MSN_Profile.legtype',W_Mat];
        FinalWeightData=array2table(FinalWeightData);
        FinalWeightData.Properties.VariableNames=["Leg Type","Starting Weight [lb]","Ending Weight [lb]", "Fuel Weight [lb]", "Weight Fraction", "Fuel Added"];

        IterationData=array2table(IterationData);
        IterationData.Properties.VariableNames=["Iteration Number","W0 [lb]","Empty Weight [lb]","Total Fuel Weight [lb]"];
        FinalSegmentData=DATA;

    end
    
    %%Data Tables for output.
    %Convert data structure to table
    %T_SizingStruct = structfun(@transpose, SizingStruct, 'UniformOutput', false); %transpose data in structues for ease of reading
    %Msn_Sizing_Table = struct2table(T_SizingStruct);

    % Added in tables for easy display purposes (changes to P/W if prop vs T/W
    % % for jet)
    if strcmp(PropType,'PROP_Fuel')||strcmp(PropType,'PROP_Turbocharged')||strcmp(PropType,'PROP_Turboprop')
        %recalculate wing loading and power to weight ratio for final W0
        WingLoading = W0_calc/Sref;
        Power_Weight_Ratio =PA_shp_sl/W0_calc;
        % construct output table
        tableNames = {'W0(lb)', 'Wf req internal(lb)', 'Wf total air refuel(lb)', 'We(lb)', 'Wing Loading(lb/ft^2)','P/W(hp/lb)'};
        outputTable = table(W0_calc,W_f_req_internal,W_f_total_refuel,We,WingLoading,Power_Weight_Ratio, 'VariableNames', tableNames);
    elseif strcmp(PropType,'PROP_Electric')
        %recalculate wing loading and power to weight ratio for final W0
        WingLoading = W0_calc/Sref;
        Power_Weight_Ratio =PA_shp_sl/W0_calc;
        % construct output table
        tableNames = {'W0(lb)', 'Battery Mass (lb)', 'We(lb)', 'Wing Loading(lb/ft^2)','P/W(hp/lb)'};
        outputTable = table(W0_calc,BatteryWeight_lb,We,WingLoading,Power_Weight_Ratio, 'VariableNames', tableNames);
    else
        %recalculate wing loading and thrust to weight ratio for final W0
        WingLoading = W0_calc/Sref;
        Thrust_Weight_Ratio_mil =TA_mil_sl/W0_calc;
        Thrust_Weight_Ratio_AB = TA_AB_sl/W0_calc;
        % construct output table
        tableNames = {'W0(lb)', 'Wf req internal(lb)', 'Wf total air refuel(lb)', 'We(lb)', 'Wing Loading(lb/ft^2)','T/W mil(lb/lb)', 'T/W AB (lb/lb)'};
        outputTable = table(W0_calc,W_f_req_internal,W_f_total_refuel,We,WingLoading,Thrust_Weight_Ratio_mil,Thrust_Weight_Ratio_AB, 'VariableNames', tableNames);
    end



end
















function v6PrintMissionBudget(mission, W0_lb, iteration, options)
% v6 diagnostic only: no changes to physical equations or solver results.
dt = mission.dt_s(:);
p = mission.P_motor_W(:);
e = dt .* p / 3600;
fprintf('\n[v6] Mission budget: iteration %d, W0=%.3f lb\n',iteration,W0_lb);
fprintf('  #  %-16s %12s %12s %12s %12s\n','Segment','Duration s','Motor W','Motor Wh','Cumul Wh');
for k=1:numel(dt)
    typ = string(mission.segmentType(k));
    fprintf('  %2d %-16s %12.1f %12.1f %12.2f %12.2f\n',k,char(typ),dt(k),p(k),e(k),sum(e(1:k)));
end
fprintf('  TOTAL: duration=%.1f s, motor energy=%.2f Wh, peak motor=%.1f W\n',sum(dt),sum(e),max(p));
if isfield(options,'P_avionics_W'), pa=options.P_avionics_W; else, pa=15; end
if isfield(options,'wiringLossFraction'), loss=options.wiringLossFraction; else, loss=0.03; end
if isfield(options,'energyMarginFactor'), margin=options.energyMarginFactor; else, margin=1.10; end
pbat=(p+pa)./(1-loss);
fprintf('  Estimated bus energy before margin=%.2f Wh, peak=%.1f W (avionics=%.1f W, wiring loss=%.3f)\n',sum(pbat.*dt)/3600,max(pbat),pa,loss);
fprintf('  Battery sizing energy margin factor=%.4f (see BatteryRequirements for actual application)\n',margin);
end

function v6PrintBatteryBudget(b,chem,options)
fprintf('[v6] Battery: mission=%.2f Wh, nominal pack=%.2f Wh, delivered=%.2f Wh, cell energy=%.2f Wh, resistive loss=%.2f Wh\n', ...
    b.missionEnergy_Wh,b.nominalPackEnergy_Wh,b.energyDelivered_Wh,b.energyFromCells_Wh,b.resistiveLoss_Wh);
fprintf('  Pack=%dS %.3f Ah, mass=%.3f kg, nominal V=%.2f, min terminal V=%.2f, cutoff V=%.2f, final SOC=%.4f\n', ...
    b.nSeries,b.requiredCapacity_Ah,b.mass_kg,b.nominalPackVoltage_V,b.minimumTerminalVoltage_V,b.requiredCutoffVoltage_V,b.finalSOC);
fprintf('  Peak=%.1f W, current=%.2f A, peak C=%.3f, RMS current=%.2f A; specific energy=%.1f Wh/kg\n', ...
    b.peakPower_W,b.peakCurrent_A,b.peakC,b.rmsCurrent_A,chem.specificEnergy_Whkg);
if isfield(b,'candidates') && istable(b.candidates)
    disp('[v6] Battery series-count candidate feasibility:');
    disp(b.candidates);
end
if isfield(chem,'minimumFinalSOC'), fprintf('  Minimum final SOC setting=%.4f\n',chem.minimumFinalSOC); end
if isfield(chem,'usableCapacityFraction'), fprintf('  Usable capacity fraction=%.4f\n',chem.usableCapacityFraction); end
end
