function fullfilename = writeSizingData(FLAG,W0,FinalSegmentData,FinalWeightData,IterationData,outputTable,DragPolar_Model,ProfileName,codeversion,outputfolder)
    if FLAG
        % Format filename
        currentTime = datetime('now');
        formatSpec = 'yyyy-MM-dd_HH-mm-ss';
        timeStampStr = string(currentTime, formatSpec);
        fileExtension = '.xlsx';
        filename = ProfileName + '_' + codeversion + "_" + timeStampStr + fileExtension; % Use string concatenation
        fullfilename = fullfile(outputfolder,filename);

        %write data to file
        writetable(outputTable,fullfilename,Sheet='Main Output')
        writetable(DragPolar_Model,fullfilename,Sheet='Drag Polar')
        writetable(FinalWeightData,fullfilename,Sheet='Mission Legs')
        writetable(IterationData,fullfilename,Sheet='Iteration Data')


        % Segment Info contains only mission legs; electric battery
        % diagnostics are appended to FinalSegmentData after those legs.
        nLeg = height(FinalWeightData);
        for i = 1:min(numel(FinalSegmentData),nLeg)
            writetable(FinalWeightData(i,1),fullfilename,Sheet="Segment Info",WriteMode="append")
            if ~isempty(FinalSegmentData{i})
                writetable(FinalSegmentData{i},fullfilename,Sheet="Segment Info",WriteMode="append",WriteVariableNames=true)
            else
                writematrix("No Data",fullfilename,Sheet="Segment Info",WriteMode="append")
            end
        end
        if numel(FinalSegmentData)>nLeg
            battery = FinalSegmentData{nLeg+1};
            if isstruct(battery)
                % Write scalar battery diagnostics only; histories and arrays
                % are not rectangular Excel table variables.
                fields = fieldnames(battery);
                name = strings(0,1); value = strings(0,1);
                for k=1:numel(fields)
                    val = battery.(fields{k});
                    if (isnumeric(val) || islogical(val)) && isscalar(val)
                        name(end+1,1)=string(fields{k}); %#ok<AGROW>
                        value(end+1,1)=string(val); %#ok<AGROW>
                    elseif (ischar(val) || isstring(val)) && isscalar(string(val))
                        name(end+1,1)=string(fields{k}); %#ok<AGROW>
                        value(end+1,1)=string(val); %#ok<AGROW>
                    end
                end
                if ~isempty(name)
                    writetable(table(name,value,'VariableNames',{'Parameter','Value'}), ...
                        fullfilename,Sheet="Electric Battery")
                end
            end
            if numel(FinalSegmentData)>=nLeg+2 && istable(FinalSegmentData{nLeg+2})
                writetable(FinalSegmentData{nLeg+2},fullfilename,Sheet="Electric Iteration Diagnostics")
            end
        end
        
    end
end