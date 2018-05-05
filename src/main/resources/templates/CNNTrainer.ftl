import logging
import mxnet as mx
<#list instances as instance>
import CNNCreator_${instance.fullName?replace(".", "_")}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log","w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list instances as instance>
    ${instance.fullName?replace(".", "_")} = CNNCreator_${instance.fullName?replace(".", "_")}.CNNCreator_${instance.fullName?replace(".", "_")}()
    ${instance.fullName?replace(".", "_")}.train(
    <#if (trainParams[instance_index])??>
       ${trainParams[instance_index]}
    </#if>
    )

</#list>