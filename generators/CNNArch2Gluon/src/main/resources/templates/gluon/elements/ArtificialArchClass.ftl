<#assign input = element.inputs[0]>
<#assign name = element.element.name>
<#assign args = element.element.arguments>
<#if mode == "ARTIFICIAL_ARCH_CLASS">
class ${name}(gluon.HybridBlock):
    def __init__(self, **kwargs):
        super(${name}, self).__init__(**kwargs)
        with self.name_scope():
${tc.include(element.element,"ARCHITECTURE_DEFINITION")}

    def hybrid_forward(self,F, ${input}):
${tc.include(element.element,"FORWARD_FUNCTION")}

        return <#list element.element.getLastAtomicElements() as el><#if el?index ==0>${tc.getName(el)}<#else>,tc.getName(el)</#if></#list>
</#if>