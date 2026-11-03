function [seg, msgs] = EvaluateElectricMissionSegment(segType, x, W_lb, Constants, Propulsion_Input, DragPolar_Model, WaveDrag_Data, msgs, busVoltageLimit_V)
%EVALUATEELECTRICMISSIONSEGMENT Evaluate one electric mission segment using
% aircraft aerodynamics -> ElectricPropulsion -> battery-side electrical power.
% Inputs x follow Mission_Profile input_table ordering. Units are those in the
% mission spreadsheet (altitude ft, range nm, loiter time hr, Mach dimensionless).

if nargin < 9 || isempty(busVoltageLimit_V), busVoltageLimit_V=Inf; end
cfg = Constants{1}; Sref_m2 = Constants{3}/(3.280839895^2); g = Constants{5}/3.280839895;
W_N = W_lb*4.448221615; seg = struct('type',string(segType),'duration_s',0, ...
    'P_motor_W',0,'P_shaft_W',0,'T_required_N',0,'T_available_N',0, ...
    'altitude_ft',NaN,'Mach',NaN,'requiredMotorVoltage_V',0,'feasible',true,'note','');

switch string(segType)
    case "Takeoff"
        alt_ft=x(1); CLmax=x(2); mu=x(3); CDx=x(4);
        [rho,a] = atmos(alt_ft/3.280839895); % SI
        Vstall=sqrt(2*W_N/(rho*CLmax*Sref_m2)); Vto=1.2*Vstall; Vavg=0.7*Vto; Mavg=Vavg/a;
        CL=CLmax; [CD,~,~,~,msgs]=DragPolar_Function(Mavg,CL,cfg,DragPolar_Model,WaveDrag_Data,msgs); CD=CD+CDx;
        D=CD*0.5*rho*Vavg^2*Sref_m2; L=CL*0.5*rho*Vavg^2*Sref_m2;
        Treq=max(D + mu*max(W_N-L,0) + 0.08*W_N, 1);
        [Tav,Pshaft,Pelec,~,~,~,msgs,~,Top,ok,VmotorReq]=ElectricPropulsion(alt_ft,Mavg,Treq,cfg,Propulsion_Input,msgs,busVoltageLimit_V);
        denom=rho*Sref_m2*CLmax*g*(Top-(D+mu*(W_N-L)));
        if ~ok || denom<=0 || Vavg<=0
            seg.feasible=false; seg.note='Takeoff propulsion/ground-roll requirement is infeasible.';
            seg.duration_s=1; seg.P_motor_W=max(Pelec,0); seg.P_shaft_W=Pshaft; seg.T_required_N=Treq; seg.T_available_N=Tav;
        else
            Sto=1.44*W_N^2/denom; seg.duration_s=max(Sto/Vavg,1); seg.P_motor_W=Pelec; seg.P_shaft_W=Pshaft;
            seg.T_required_N=Treq; seg.T_available_N=Tav;
        end
        seg.requiredMotorVoltage_V=VmotorReq; seg.altitude_ft=alt_ft; seg.Mach=Mavg;
    case "Climb"
        h1=x(1); h2=x(2); M1=x(3); M2=x(4); CDx=x(5); h=(h1+h2)/2; M=(M1+M2)/2;
        [rho,a]=atmos(h/3.280839895); V=max(M*a,1e-3); CL=W_N/(0.5*rho*V^2*Sref_m2);
        [CD,~,~,~,msgs]=DragPolar_Function(M,CL,cfg,DragPolar_Model,WaveDrag_Data,msgs); CD=CD+CDx;
        D=CD*0.5*rho*V^2*Sref_m2; Treq=D+W_N*sin(5*pi/180);
        [Tav,Pshaft,Pelec,~,~,~,msgs,~,Top,ok,VmotorReq]=ElectricPropulsion(h,M,Treq,cfg,Propulsion_Input,msgs,busVoltageLimit_V);
        % The mission currently prescribes a 5-degree climb path. Propulsive
        % power is thrust times airspeed, NOT motor shaft power. Use the
        % prescribed climb angle to determine elapsed time and electrical
        % energy; do not infer an inconsistent climb time from shaft power.
        gamma=5*pi/180;
        propulsivePower_W=Top*V;
        excessPropulsivePower_W=propulsivePower_W-D*V;
        requiredClimbPower_W=W_N*V*sin(gamma);
        ROC=V*sin(gamma);
        seg.duration_s=max(abs(h2-h1)*0.3048/max(ROC,eps),1);
        seg.climbExcessPower_W=excessPropulsivePower_W;
        seg.climbRequiredPower_W=requiredClimbPower_W;
        seg.climbROC_mps=ROC;
        if ~ok || ~isfinite(Top) || ...
                excessPropulsivePower_W + 1e-6 < requiredClimbPower_W
            seg.feasible=false;
            seg.note=sprintf(['Climb thrust/power infeasible: required %.2f N, ' ...
                'operating %.2f N, excess propulsive %.1f W, required climb %.1f W.'], ...
                Treq,Top,excessPropulsivePower_W,requiredClimbPower_W);
        end
        seg.P_motor_W=max(Pelec,0); seg.P_shaft_W=Pshaft; seg.T_required_N=Treq; seg.T_available_N=Tav; seg.requiredMotorVoltage_V=VmotorReq; seg.altitude_ft=h; seg.Mach=M;
    case {"Cruise","Supercruise"}
        h=x(1); range_nm=x(2); M=x(3); CDx=x(4); [rho,a]=atmos(h/3.280839895); V=M*a;
        if V<=0, error('Cruise Mach must be positive.'); end
        CL=W_N/(0.5*rho*V^2*Sref_m2); [CD,~,~,~,msgs]=DragPolar_Function(M,CL,cfg,DragPolar_Model,WaveDrag_Data,msgs); CD=CD+CDx;
        D=CD*0.5*rho*V^2*Sref_m2; [Tav,Pshaft,Pelec,~,~,~,msgs,~,~,ok,VmotorReq]=ElectricPropulsion(h,M,D,cfg,Propulsion_Input,msgs,busVoltageLimit_V);
        seg.duration_s=max(range_nm*1852/V,1); seg.P_motor_W=max(Pelec,0); seg.P_shaft_W=Pshaft; seg.T_required_N=D; seg.T_available_N=Tav;
        seg.feasible=ok; if ~ok, seg.note='Cruise thrust requirement exceeds available propulsion.'; end
        seg.requiredMotorVoltage_V=VmotorReq; seg.altitude_ft=h; seg.Mach=M;
    case "Loiter"
        h=x(1); t_hr=x(2); CDx=x(3); [rho,a]=atmos(h/3.280839895);
        CDo=DragPolar_Model.CDo_sub(cfg); k=DragPolar_Model.k1_sub(cfg); CL=sqrt(3*CDo/k); V=sqrt(2*W_N/(rho*Sref_m2*CL)); M=V/a;
        [CD,~,~,~,msgs]=DragPolar_Function(M,CL,cfg,DragPolar_Model,WaveDrag_Data,msgs); CD=CD+CDx;
        D=CD*0.5*rho*V^2*Sref_m2; [Tav,Pshaft,Pelec,~,~,~,msgs,~,~,ok,VmotorReq]=ElectricPropulsion(h,M,D,cfg,Propulsion_Input,msgs,busVoltageLimit_V);
        seg.duration_s=max(t_hr*3600,1); seg.P_motor_W=max(Pelec,0); seg.P_shaft_W=Pshaft; seg.T_required_N=D; seg.T_available_N=Tav;
        seg.feasible=ok; if ~ok, seg.note='Loiter thrust requirement exceeds available propulsion.'; end
        seg.requiredMotorVoltage_V=VmotorReq; seg.altitude_ft=h; seg.Mach=M;
    case {"Startup/Taxi","Descent","Landing"}
        % The current mission template provides only a legacy weight fraction
        % for these phases, not altitude/speed/time. Use documented workbook
        % estimates until detailed segment conditions are supplied.
        opt=Propulsion_Input.ElectricAuxSegmentDefaults{cfg};
        switch string(segType)
            case "Startup/Taxi", key='StartupTaxi';
            case "Descent", key='Descent';
            otherwise, key='Landing';
        end
        seg.duration_s=opt.([key '_duration_s']);
        seg.P_motor_W=opt.([key '_powerFraction'])*Propulsion_Input.MotorData{cfg}.Pmax_W*Propulsion_Input.Number(cfg);
        seg.note='Estimated phase power/time from workbook defaults; replace when detailed segment conditions are available.';
    otherwise
        error('Electric mission segment type "%s" is not supported by the integrated model.',string(segType));
end
if ~isfinite(seg.P_motor_W) || seg.P_motor_W<0 || ~isfinite(seg.duration_s) || seg.duration_s<=0
    error('Invalid duration/power returned for electric mission segment %s.',string(segType));
end
end
