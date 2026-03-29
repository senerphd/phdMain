%% Clear screen & data
d_05    = allData.Hasan_PhD_RMC200_all_0_20260107_193503929.Data;
d_1     = allData.Hasan_PhD_RMC200_all_0_20260107_193718658.Data;
d_15    = allData.Hasan_PhD_RMC200_all_0_20260107_193857313.Data;
d_2     = allData.Hasan_PhD_RMC200_all_0_20260107_193948059.Data;
d_25    = allData.Hasan_PhD_RMC200_all_0_20260107_194035435.Data;
d_3     = allData.Hasan_PhD_RMC200_all_0_20260107_194118538.Data;
ss      = allData.Hasan_PhD_RMC200_all_0_20260107_195012267.Data; 
allDataCell = struct2cell(allData); 
numOfData = length(allDataCell); 

%% Select Test Data 
sData = d_1 ;
test_data = sData; 

clc
generateTimeSeriesVariables(sData);
constants; 
close all 


%% Calculate Friction 
close all 
figure; 
legendCell = cell(1,numOfData); 
figure;
for i = 1:numOfData
    data_ = allDataCell{i,1}.Data;
    plot(data_.v_mPs,data_.F_friction,'.')
    hold on
    legendCell{i} = strcat(allDataCell{i,1}.Waveform,'-',num2str(allDataCell{i,1}.Frequency),'-',allDataCell{i,1}.Rate);
end
legend(legendCell)
xlabel('v')
ylabel('F_r')
grid on 

figure; 
for i = 1:numOfData
    data_ = allDataCell{i,1}.Data;
    plot(data_.t,data_.F_friction,'-')
    hold on
    legendCell{i} = strcat(allDataCell{i,1}.Waveform,'-',num2str(allDataCell{i,1}.Frequency),'-',allDataCell{i,1}.Rate);
end
legend(legendCell)
xlabel('t')
ylabel('F_r')

figure; 
for i = 1:numOfData
    data_ = allDataCell{i,1}.Data;
    plot(data_.t,data_.v_mPs,'-')
    hold on
    legendCell{i} = strcat(allDataCell{i,1}.Waveform,'-',num2str(allDataCell{i,1}.Frequency),'-',allDataCell{i,1}.Rate);
end
legend(legendCell)
xlabel('t')
ylabel('v')

grid on 
%% 
clc
generateTimeSeriesVariables(sData);
constants; 
close all 
