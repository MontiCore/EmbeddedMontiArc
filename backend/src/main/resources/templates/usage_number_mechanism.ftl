<#macro usage_number_mechanism event id maxUsages>
    <mechanism event='urn:action:rwth-student-solution:${event}'>
        <if>
            <and>
                <equals>
                    <event:string eventParameter='datasetId' default='' jsonPathQuery='$.id'/>
                    <constant:string value='${id}'/>
                </equals>
                <greaterEqual>
                    <count>
                        <eventOccurrence event='urn:action:rwth-student-solution:dataset-access'>
                            <parameter:string name='dataset-id' value='${id}'/>
                        </eventOccurrence>
                        <when fixedTime='always'/>
                    </count>
                    <constant:number value='${maxUsages}'/>
                </greaterEqual>
            </and>
            <then>
                <inhibit/>
                <execute action='urn:action:rwth-student-solution:delete-dataset'>
                    <parameter:string name='id'>
                        <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>
                    </parameter:string>
                </execute>
            </then>
        </if>
    </mechanism>
</#macro>
