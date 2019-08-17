<#list tc.getLayerVariableMembers("batch_size")?keys as member>
                    ${member} = mx.nd.zeroes((${tc.join(tc.getLayerVariableMembers("batch_size")[member], ", ")},), ctx=mx_context)
</#list>
<#list tc.architecture.outputs as output>
                    ${tc.getName(output)} = mx.nd.zeroes(((${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#list>

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(stream), ", ")} = self._networks[${stream?index}](${tc.join(tc.getStreamInputNames(stream), ", ")})
<#else>
${tc.include(stream, "PYTHON_INLINE")}
</#if>
</#list>