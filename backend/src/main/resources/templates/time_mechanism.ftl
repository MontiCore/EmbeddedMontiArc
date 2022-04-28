<#macro time_mechanism event timeRule>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <or>
                <#if timeRule.before??>
                <time is='before' value='${timeRule.getBefore().toString()}'/>
                </#if>
                <#if timeRule.after??>
                <time is='after' value='${timeRule.getAfter().toString()}'/>
                </#if>
            </or>
            <then>
                <inhibit/>
            </then>
        </if>
    </mechanism>
</#macro>
