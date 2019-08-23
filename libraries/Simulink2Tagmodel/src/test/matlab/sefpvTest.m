// (c) https://github.com/MontiCore/monticore 
% add required paths
%   1. the tool 
addpath('../../main/matlab/SEFPV');
%   2. the test files
addpath([pwd '/SEFPVTest']);
%   3. the resources
addpath('../resources/');

% load test system
systemName = 'Oeffentlicher_Demonstrator_FAS_v04';
load_system(systemName);
display(' ');

% run tests
getComponentOutputsTest;
getPathsToBlockTest;
computeWCETTest;
computeBCETTest;
% computeEnergyTest;

display('------------------------------');
display('All tests passed.');
% close test system
close_system(systemName);
