<#-- (c) https://github.com/MontiCore/monticore -->
<#assign input = element.inputs[0]>
<#assign ratio = element.p?c?string>
            ${element.name} = brew.dropout(model, ${input}, '${element.name}', ratio=${ratio}, is_test=False)
