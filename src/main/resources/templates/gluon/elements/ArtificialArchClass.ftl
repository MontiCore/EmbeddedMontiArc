<#assign input = element.inputs[0]>
<#if mode == "ARTIFICIAL_ARCH_CLASS" >
<#assign name = element.element.name>
<#assign args = element.element.arguments>

class artificial_layer_${name}(gluon.HyrbidBlock):
    def __init__(self, <#list args as arg>${arg}, </#list>, **kwargs):
        super(artificial_layer_${name}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(element.element,"ARCHITECTURE_DEFINITION")}
            pass

    def hybrid_forward(self, ${input}):
${tc.include(element.element,"FORWARD_FUNCTION")}

        return [[<#list element.element.getLastAtomicElements() as el><#if el?index ==0>${tc.getName(el)}<#else>,tc.getName(el)</#if></#list>]]




</#if>



