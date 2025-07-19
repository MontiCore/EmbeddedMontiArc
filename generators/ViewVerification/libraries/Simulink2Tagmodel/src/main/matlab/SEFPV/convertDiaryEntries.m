// (c) https://github.com/MontiCore/monticore 
function handles = convertDiaryEntries(diaryPath)
    file = fopen(diaryPath);
    out = [];
    
    while ~feof(file)
        line = fgets(file);
        tmp = strsplit(line, '''');
        
        if(numel(tmp) > 1)
            path = tmp{2};
            
            if ~isempty(strfind(path,'Oeffentlicher'))
                try
                    handle = get_param(path, 'Handle');
                    out = [out; handle];
                catch
                end
            end
        end        
    end
    
    fclose(file);
    handles = out;
end
