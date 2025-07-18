<#-- (c) https://github.com/MontiCore/monticore -->
        <#assign dimensions = element.element.outputTypes[0].dimensions[1..] + [element.element.outputTypes[0].dimensions[0]]>
        # ${element.name}, output shape: {[${tc.join(dimensions, ",")}]}

        
