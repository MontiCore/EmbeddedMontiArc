<#if mode == "ADANET_CONSTRUCTION">
<#assign outBlock = element.element.getDeclaration().getBlock("outBlock")>
<#assign inBlock = element.element.getDeclaration().getBlock("inBlock")>
<#assign Block = element.element.getDeclaration().getBlock("block").get()>
<#if Block.isArtificial()>
#BuildingBlock
</#if>
${tc.include(Block,"ARTIFICIAL_ARCH_CLASS")}
<#if inBlock.isPresent()>
#inputBlock
<#if inBlock.get().isArtificial()>
${tc.include(inBlock.get(),"ARTIFICIAL_ARCH_CLASS")}
</#if>
</#if>
<#if outBlock.isPresent()>
<#if outBlock.get().isArtificial()>
#outputBlock
${tc.include(outBlock.get(),"ARTIFICIAL_ARCH_CLASS")}
</#if>
</#if>
</#if>