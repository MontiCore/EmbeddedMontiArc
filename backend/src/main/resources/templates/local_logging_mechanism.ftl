<#macro local_logging_mechanism event>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <constant:true/>
            <then>
                <execute action='urn:action:rwth-student-solution:log-local'>
                    <parameter:string name='url'>
                        <event:string eventParameter='datasetId' default='' jsonPathQuery='$.id'/>
                    </parameter:string>
                </execute>
            </then>
        </if>
    </mechanism>
</#macro>
