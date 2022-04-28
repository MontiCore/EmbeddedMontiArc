<#macro remote_logging_mechanism event>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <constant:true/>
            <then>
                <execute action='urn:action:rwth-student-solution:log-remote'>
                    <parameter:string name='url'>
                        <event:string eventParameter='dataset' default='' jsonPathQuery='$.loggingUrl'/>
                    </parameter:string>
                    <parameter:string name='id'>
                        <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>
                    </parameter:string>
                </execute>
            </then>
        </if>
    </mechanism>
</#macro>
