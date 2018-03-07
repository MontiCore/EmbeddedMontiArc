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
    ${instance.fullName?replace(".", "_")}.train(batch_size=100,
        num_epoch=10,
        optimizer='sgd',
        optimizer_params={'learning_rate':0.1},
        load_checkpoint=True)
</#list>