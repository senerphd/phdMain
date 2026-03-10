clear all
clc
%% Add Library Path 
addpath('.\_lib\');
%% Import Data 
% Define folder location 
folderPath = 'C:\_PhD\4_Tez\3_Tests\1_TestData\20260107\txtData';
allData = importAllData(folderPath);
%% Select Data 
analyzeData_v3; 

%% Open model 
open_system('phdMain.slx');



