<#macro local_logging_mechanism event id>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <equals>
                <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>
                <constant:string value='${id}'/>
            </equals>
            <then>
                <execute action='urn:action:rwth-student-solution:log-local'>
                    <parameter:string name='id'>
                        <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>
                    </parameter:string>
                </execute>
            </then>
        </if>
    </mechanism>
</#macro>
