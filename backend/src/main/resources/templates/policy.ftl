<#import "time_mechanism.ftl" as t_macro>
<#import "local_logging_mechanism.ftl" as ll_macro>
<#import "remote_logging_mechanism.ftl" as rl_macro>
<#import "usage_number_mechanism.ftl" as u_macro>

<policy id='urn:policy:rwth-student-solution:${id}'
        xmlns='http://www.mydata-control.de/4.0/mydataLanguage'
        xmlns:parameter='http://www.mydata-control.de/4.0/parameter'
        xmlns:constant='http://www.mydata-control.de/4.0/constant'
>
    <#if startTime?? || endTime()??>
    <@t_macro.time_mechanism event=event id=id startTime=startTime endTime=endTime/>
    </#if>
    <#if maxUsages??>
    <@u_macro.usage_number_mechanism event=event id=id maxUsages=maxUsages/>
    </#if>
    <#if localLogging>
    <@rl_macro.remote_logging_mechanism event=event id=id/>
    </#if>
    <#if remoteLogging>
    <@ll_macro.local_logging_mechanism event=event id=id/>
    </#if>
</policy>
