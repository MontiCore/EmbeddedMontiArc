<#import "time_mechanism.ftl" as t_macro>
<#import "local_logging_mechanism.ftl" as ll_macro>
<#import "remote_logging_mechanism.ftl" as rl_macro>
<#import "usage_number_mechanism.ftl" as u_macro>

<policy id='urn:policy:rwth-student-solution:${id}'
        xmlns='http://www.mydata-control.de/4.0/mydataLanguage'
        xmlns:parameter='http://www.mydata-control.de/4.0/parameter'
        xmlns:constant='http://www.mydata-control.de/4.0/constant'
        xmlns:event='http://www.mydata-control.de/4.0/event'

>
    <#if businessHours.getStart()?? || businessHours.getEnd()??>
        <@t_macro.time_mechanism event=event id=id businessHours=businessHours/>
    </#if>
    <#if maxUsages??>
        <@u_macro.usage_number_mechanism event=event id=id maxUsages=maxUsages/>
    </#if>
    <#if localLogging>
        <@ll_macro.local_logging_mechanism event=event id=id/>
    </#if>
    <#if remoteLogging>
        <@rl_macro.remote_logging_mechanism event=event id=id/>
    </#if>
</policy>
