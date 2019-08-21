<#list tc.getLayerVariableMembers("batch_size")?keys as member>
                    ${member} = mx.nd.zeros((${tc.join(tc.getLayerVariableMembers("batch_size")[member], ", ")},), ctx=mx_context)
</#list>
<#list tc.architecture.outputs as output>
                    ${tc.getName(output)} = mx.nd.zeros((${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(stream), ", ")} = self._networks[${stream?index}](${tc.join(tc.getStreamInputNames(stream), ", ")})
<#else>
${tc.include(stream, "PYTHON_INLINE")}
</#if>
</#list>

<#list tc.architecture.unrolls as unroll>
<#if unroll.isTrainable()>
                    ${tc.join(tc.getUnrollOutputNames(unroll), ", ")} = self._networks[${unroll?index}](${tc.join(tc.getUnrollInputNames(unroll), ", ")})
<#else>
${tc.include(unroll, "PYTHON_INLINE")}
</#if>
</#list>