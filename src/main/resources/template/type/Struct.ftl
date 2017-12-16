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

#endif
