<#macro time_mechanism event id businessHours>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <and>
                <equals>
                    <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>
                    <constant:string value='${id}'/>
                </equals>
                <or>
                    <#if businessHours.getStart()??>
                        <time is='before' value='${businessHours.getStart().toString()}'/>
                    </#if>
                    <#if businessHours.getEnd()??>
                        <time is='after' value='${businessHours.getEnd().toString()}'/>
                    </#if>
                </or>
            </and>
            <then>
                <inhibit/>
            </then>
        </if>
    </mechanism>
</#macro>
