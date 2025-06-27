<#-- (c) https://github.com/MontiCore/monticore -->
<#include "/Common.ftl">
#ifndef ${viewModel.includeName?upper_case}
#define ${viewModel.includeName?upper_case}

enum ${viewModel.includeName} {
<#list viewModel.constants as ec>
${ec}<#sep>,</#sep>
</#list>
};

#endif
