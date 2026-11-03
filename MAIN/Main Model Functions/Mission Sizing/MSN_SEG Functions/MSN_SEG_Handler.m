function [W_Mat,BMF_Mat,DATA,msgs,mission] = MSN_SEG_Handler(MSN_Profile,W_start,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs,busVoltageLimit_V)
% Mission segment dispatcher. Electric aircraft use mission -> propeller/motor
% operating points -> mission electrical power profile. Non-electric logic is
% retained from the original model.
if nargin < 9 || isempty(busVoltageLimit_V),busVoltageLimit_V=Inf;end
legnum=MSN_Profile.legnum; input_table=MSN_Profile.input_table; segment=MSN_Profile.legtype;
W=W_start; W_Mat=zeros(legnum,5); config=Constants{1}; PropType=string(Propulsion_Input.PropType{config});
BMF_Mat=zeros(1,legnum); mission=struct('dt_s',[],'P_motor_W',[],'segmentType',strings(0,1),'segmentData',{{}},'Vmin_motor_V',[]);
DATA=cell(1,legnum);
if PropType=="PROP_Electric"
    energyWh=zeros(1,legnum);
    for i=1:legnum
        seg_inpt=input_table{i}; segType=string(segment(i));
        if segType=="Drop" || segType=="VTOL" || segType=="Air Refuel"
            error('Electric integration currently does not support mission segment "%s". Update the segment template before running.',segType);
        end
        [seg,msgs]=EvaluateElectricMissionSegment(segType,seg_inpt,W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs,busVoltageLimit_V);
        if ~seg.feasible
            fprintf(2,'[v6] Failed segment %d (%s): duration=%.1f s, required thrust=%.2f N, available thrust=%.2f N, shaft=%.1f W, electrical=%.1f W\n', ...
                i,char(segType),seg.duration_s,seg.T_required_N,seg.T_available_N,seg.P_shaft_W,seg.P_motor_W);
            error('Electric mission segment %d (%s) is infeasible: %s',i,segType,seg.note);
        end
        if isfield(seg,'note') && ~isempty(seg.note)
            msgs.warnings{end+1}=sprintf('Segment %d (%s): %s',i,segType,seg.note);
        end
        seg.ElectricalEnergy_Wh=seg.P_motor_W*seg.duration_s/3600;
        DATA{i}=struct2table(seg,'AsArray',true);
        mission.dt_s(end+1,1)=seg.duration_s; %#ok<AGROW>
        mission.P_motor_W(end+1,1)=seg.P_motor_W; %#ok<AGROW>
        mission.Vmin_motor_V(end+1,1)=seg.requiredMotorVoltage_V; %#ok<AGROW>
        mission.segmentType(end+1,1)=segType; %#ok<AGROW>
        mission.segmentData{end+1,1}=seg; %#ok<AGROW>
        energyWh(i)=seg.ElectricalEnergy_Wh;
        BMF_Mat(i)=0; % legacy column retained; battery mass is now calculated once per mission
    end
    totalE=sum(energyWh);
    if totalE>0, BMF_Mat=energyWh/totalE; end % segment energy fractions for reporting only
else% NON-ELECTRIC
        for i=1:legnum%iterates through the given mission profile and calls the relevant MSN_SEG function for each legtype.
            W_Mat(i,1)=W; %starting weight
            seg_inpt=input_table{i};%gets the row for the ith mission segment from the cell array of inputs.
            dropflag=false;%initial flag for when the aircraft is dropping payload in segment
            refuelflag=false;% initial flag for if air refueling segment present in mission profile
            switch segment(i)
                case "Takeoff"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_TO(seg_inpt(1),seg_inpt(2),seg_inpt(3),seg_inpt(4),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
                case "Climb"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_Climb(seg_inpt(1),seg_inpt(2),seg_inpt(3),seg_inpt(4),seg_inpt(5),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
                case "Cruise"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_Cruise(seg_inpt(1),seg_inpt(2),seg_inpt(3),seg_inpt(4),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
                case "Supercruise"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_Supercruise(seg_inpt(1),seg_inpt(2),seg_inpt(3),seg_inpt(4),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
                case "Loiter"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_Loiter(seg_inpt(1),seg_inpt(2),seg_inpt(3),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);
                case "Startup/Taxi"
                    Wa_Wb=seg_inpt(1);
                    Weight_Fraction=Wa_Wb;
                    DATA{i}=table(Weight_Fraction);
                case "Descent"
                    Wa_Wb=seg_inpt(1);
                    Weight_Fraction=Wa_Wb;
                    DATA{i}=table(Weight_Fraction);
                case "Landing"
                    Wa_Wb=seg_inpt(1);
                    Weight_Fraction=Wa_Wb;
                    DATA{i}=table(Weight_Fraction);
                case "Drop"
                    Wa_Wb=(W-seg_inpt(1))/W;
                    dropflag=true;
                    Payload_Dropped=seg_inpt(1);
                    Weight_Fraction=Wa_Wb;
                    DATA{i}=table(Payload_Dropped,Weight_Fraction);
                case "VTOL"
                    [Wa_Wb,DATA{i},msgs]=MSN_SEG_VTOL(seg_inpt(1),seg_inpt(2),seg_inpt(3),W,Constants,Propulsion_Input,DragPolar_Model,WaveDrag_Data,msgs);

                case "Air Refuel"
                    Wa_Wb=(W+seg_inpt(1))/W;
                    refuelflag=true;
                    Fuel_Added=seg_inpt(1);
                    Weight_Fraction=Wa_Wb;
                    DATA{i}=table(Fuel_Added,Weight_Fraction);
                otherwise
                    error("Uknown Mission Segment Type:");
            end

            if dropflag

                %setsfuel weight to zero if the aircraft had a segment that
                %was dropping payload or air refueling
                %(these segments are instantaneous paylaod drops so no fuel is used)
                W_Fuel=0;
                Fuel_Added=0;
            elseif refuelflag
                W_Fuel=0;
            else
                W_Fuel=W-Wa_Wb*W;
                Fuel_Added = 0;
            end
            W=W*Wa_Wb;
            W_Mat(i,2)=W;%ending weight
            W_Mat(i,3)=W_Fuel; %Fuel used during segment
            W_Mat(i,4)=Wa_Wb; %Segment weight fraction
            W_Mat(i,5)=Fuel_Added; %Fuell added during segment

        end

    end


end

