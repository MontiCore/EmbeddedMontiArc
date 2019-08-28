<#list tc.getLayerVariableMembers("batch_size")?keys as member>
                    ${member} = mx.nd.zeros((${tc.join(tc.getLayerVariableMembers("batch_size")[member], ", ")},), ctx=mx_context)
</#list>
<#list tc.architecture.outputs as output>
<#if tc.getName(output)??>
                    ${tc.getName(output)} = mx.nd.zeros((${tc.join(output.ioDeclaration.type.dimensions, ", ")},), ctx=mx_context)
</#if>
</#list>

<#list tc.architecture.streams as stream>
<#if stream.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(stream), ", ")} = self._networks[${stream?index}](${tc.join(tc.getStreamInputNames(stream), ", ")})
<#else>
${tc.include(stream, "PYTHON_INLINE")}
</#if>
</#list>

<#list tc.architecture.unrolls as unroll>
<#list unroll.getBodiesForAllTimesteps() as body>
<#if body.isTrainable()>
                    ${tc.join(tc.getStreamOutputNames(body), ", ")} = self._networks[${tc.architecture.streams?size + body?index}](${tc.join(tc.getStreamInputNames(body), ", ")})
<#else>
${tc.include(unroll, true, "PYTHON_INLINE")}
</#if>
</#list>
</#list>