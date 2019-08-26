<#-- (c) https://github.com/MontiCore/monticore -->
<#include "/Common.ftl">
#ifndef ${viewModel.includeName?upper_case}
#define ${viewModel.includeName?upper_case}

<#list viewModel.additionalIncludes as inc>
#include "${inc}.h"
</#list>

struct ${viewModel.includeName} {
<#list viewModel.fields as f>
${f.type} ${f.name}<#if f.initializer?has_content> = ${f.initializer}</#if>;
</#list>
};

std::ostream& operator<<(std::ostream& os, const ${viewModel.includeName}& s)
{
    os << '{';
    <#list viewModel.fields as f>
    os << "${f.name}: " << s.${f.name};
    <#if f?has_next>os << ", ";</#if>
    </#list>
    os << '}';
    return os;
}

void toFileString(std::ofstream& myfile, const ${viewModel.includeName}& s){
    myfile << s;
}


#endif
