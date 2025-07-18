// (c) https://github.com/MontiCore/monticore 
function s2tm(cmp, tagType, package, outputDir, slistFile)
%S2TM creates a tag model for the provided Simulink block path
%   Allowed paths are root level, subsystem or atomic block paths paths of
%   a Simulink model. Note that the Simulink model needs to be open at the
%   time this method is invoked.
    if ~exist('tagType', 'var')
        error('Please provide a tag type.');
    end
    if ~exist('slistFile', 'var')
        slistFile = '';
    end
    if ~exist('outputDir', 'var')
        outputDir = getDefaultPath;
    end
    if ~exist('package', 'var')
        package = getDefaultPackage(outputDir);
    end
    addpath([fileparts(mfilename('fullpath')) filesep 'S2TM']);
    % add path of the OO Implementation
    addpath('SEFPV/OOImplementation');
    % add the path of the NFP values 
    addpath('../resources/NFP Values');
    
    packageStr = createPackage(package);
    conformsStr = getConformsString(tagType);
    tmHeader = [packageStr conformsStr];
    
    indent = '';

    if strcmp(tagType,'Latency')                
        nfpObj = WCET('WCET.txt');
    elseif strcmp(tagType, 'CompPower')
        nfpObj = CompPower('CompPower.txt');
    else
        Error('Tag type not supported');
    end
    
    calc = NFPCalculator(nfpObj);
    tagmodel = [tmHeader startTagModel(cmp, calc, slistFile, indent) endTagModel];
        
    fprintf('Finished computing tags.\n');
    fprintf('Writing output...\n');
    writeOutput(tagmodel, outputDir, cmp, tagType);
end

function writeOutput(tm, outputDir, cmp, tagType)
    fid = fopen([outputDir filesep Util.getBlockName(cmp) '_' tagType '.tag'],'wt');
    
    % remove blank lines from tm
    while ~isempty(strfind(tm, '\n\n'))
        tm = strrep(tm, '\n\n', '\n');
    end
    
    % print to file
    fprintf(fid, '%s', sprintf(tm));
    fclose(fid);
    fprintf('Finished.\n');

    % print to console
    % fprintf(tm);
end
