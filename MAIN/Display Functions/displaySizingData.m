function  displaySizingData(FLAG,W0,FinalSegmentData,FinalWeightData,IterationData,outputTable,MSN_Profile)
    disp("================================================================================")
    disp("W0: "+ num2str(W0) + " lbs")
    disp("================================================================================")


    if FLAG
        disp(IterationData)
        disp("================================================================================")
        disp(FinalWeightData)
        disp("================================================================================")

        % Electric sizing appends battery summary and iteration diagnostics
        % after the mission legs. Do not index the leg table past its height.
        nLeg = height(FinalWeightData);
        for i=1:min(numel(FinalSegmentData),nLeg)
            disp(table2array(FinalWeightData(i,1)))
            disp(FinalSegmentData{i})
        end
        if numel(FinalSegmentData)>nLeg
            disp('Electric battery and sizing diagnostics:')
            for i=nLeg+1:numel(FinalSegmentData)
                disp(FinalSegmentData{i})
            end
        end
        disp("================================================================================")
        disp(outputTable)
        disp("================================================================================")
        if (size(FinalWeightData,2)>2) %only plots fuelburn for non-electric

            figure
            fuelburn=FinalWeightData{:,4};
            fuelburn=str2double(fuelburn);
            bar(fuelburn,'Labels',MSN_Profile.legtype(:))
            xlabel("Mission Segment")
            ylabel("Fuel Burn [lbf]")
            title("Fuel Burn over Mission Segments")
            grid on
        end
    end

end

