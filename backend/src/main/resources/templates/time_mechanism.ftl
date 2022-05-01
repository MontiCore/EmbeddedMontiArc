<#macro time_mechanism event id startTime endTime>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <and>
                <equals>
                    <event:string eventParameter='datasetId' default='' jsonPathQuery='$.id'/>
                    <constant:string value='${id}'/>
                </equals>
                <or>
                    <#if startTime??>
                    <time is='before' value='${startTime.toString()}'/>
                    </#if>
                    <#if endTime??>
                    <time is='after' value='${endTime.toString()}'/>
                    </#if>
                </or>
            </and>
            <then>
                <inhibit/>
            </then>
        </if>
    </mechanism>
</#macro>
