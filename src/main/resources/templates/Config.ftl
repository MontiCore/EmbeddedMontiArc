<#if (tc.batchSize)??>
batch_size = ${tc.batchSize},
</#if>
<#if (tc.numEpoch)??>
num_epoch = ${tc.numEpoch},
</#if>
<#if (tc.loadCheckpoint)??>
load_checkpoint = ${tc.loadCheckpoint?string("True","False")},
</#if>
<#if (tc.context)??>
context = '${tc.context}',
</#if>
<#if (tc.normalize)??>
normalize = ${tc.normalize?string("True","False")},
</#if>
<#if (tc.evalMetric)??>
eval_metric = '${tc.evalMetric}',
</#if>
<#if (tc.configuration.optimizer)??>
optimizer = '${tc.optimizerName}',
optimizer_params = {
    <#list tc.optimizerParams?keys as param>
    '${param}': ${tc.optimizerParams[param]}<#sep>,
    </#list>
}
</#if>


