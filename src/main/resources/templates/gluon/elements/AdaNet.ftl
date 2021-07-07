<#assign input = element.inputs[0]>


<#if mode == "ARCHITECTURE_DEFINITION">
            self.${element.name} = AdaNet(block=${element.element.getDeclaration().getBlock("block").get().getName()}, in = buff, out = bufff)
            ${element.element.getDeclaration().getBlock("block").get().getName()}
<#elseif mode == "FORWARD_FUNCTION">
    ${element.name} = self.${element.name}(${input})


<#elseif mode == "ADANET_CONSTRUCTION">
    <#assign outBlock = element.element.getDeclaration().getBlock("outBlock").get()>
    <#assign inBlock = element.element.getDeclaration().getBlock("inBlock").get()>
    <#assign Block = element.element.getDeclaration().getBlock("block").get()>
# Block generation
${tc.include(Block,"ARTIFICIAL_ARCH_CLASS")}
    # Inblock generation
${tc.include(inBlock,"ARTIFICIAL_ARCH_CLASS")}

<#if outBlock.isArtificial()>
    ${outBlock.name}
    ${tc.include(outBlock,"ARTIFICIAL_ARCH_CLASS")}
</#if>
class AdaNet(gluon.HybridBlock):
    def __init(self,data_mean=None,data_std=None,mx_context=None,**kwargs):
        super(AdaNet,self).__init__(**kwargs)
        self.block = ${tc.include(Block,"ARCHITECTURE_DEFINITION")}
        <#if inBlock.isArtificial()>
        self.in = ${tc.include(inBlock,"ARCHITECTURE_DEFINITION")}
        <#else>
        self.in = None
        </#if>
        <#if inBlock.isArtificial()>
        self.in = ${tc.include(inBlock,"ARCHITECTURE_DEFINITION")}
        <#else>
        self.out = None
        </#if>

    def hybrid_forward(self, F, #{input}):
</#if>

