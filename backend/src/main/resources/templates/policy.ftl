<#import "time_mechanism.ftl" as t_macro>
<#import "local_logging_mechanism.ftl" as ll_macro>
<#import "remote_logging_mechanism.ftl" as rl_macro>

<policy id='urn:policy:rwth-student-solution:${id}'
        xmlns='http://www.mydata-control.de/4.0/mydataLanguage'
>
    <#if timeRule??>
    <@t_macro.time_mechanism event=event timeRule=timeRule/>
    </#if>
    <#if remoteLogging>
    <@rl_macro.remote_logging_mechanism event=event/>
    </#if>
    <#if localLogging>
    <@ll_macro.local_logging_mechanism event=event/>
    </#if>
</policy>
