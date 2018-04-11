import logging

<#list componentNames as name>
import CNNCreator_${name?replace(".", "_")}
</#list>

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log","w", encoding=None, delay="true")
    logger.addHandler(handler)

<#list instances as instance>
    ${instance.fullName?replace(".", "_")} = CNNCreator_${instance.componentType.fullName?replace(".", "_")}.CNNCreator_${instance.componentType.fullName?replace(".", "_")}()
    ${instance.fullName?replace(".", "_")}.train(
    <#if (trainParams[instance_index])??>
       ${trainParams[instance_index]}
    </#if>
    )

</#list>