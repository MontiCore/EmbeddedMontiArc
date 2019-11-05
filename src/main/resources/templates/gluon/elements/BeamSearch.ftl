<#--                BeamSearchPredictions = [
<#list tc.architectureOutputs as output_name>
                    mx.nd.topk(${output_name}, axis=1, k=4)<#sep>,
</#list>
                ]

                logging.info("BeamSearch indices: " + str(BeamSearchPredictions))-->