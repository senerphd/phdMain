clear all
clc
%% Add Library Path 
addpath('.\_lib\');
%% Import Data 
% Define folder location 
% folderPath = 'C:\_PhD\4_Tez\3_Tests\1_TestData\20260107\txtData';
% allData = importAllData(folderPath);

[testData] = importfile4('C:\_PhD\4_Tez\3_Tests\1_TestData\20260326\txtData\RMC200_all_2.txt');

% [testData] = importfile4('C:\_PhD\4_Tez\3_Tests\1_TestData\20260107\txtData\Hasan_PhD_RMC200_all_0_20260107_194118538.txt');


constants; 
%% Open model 
open_system('phdMain.slx');



