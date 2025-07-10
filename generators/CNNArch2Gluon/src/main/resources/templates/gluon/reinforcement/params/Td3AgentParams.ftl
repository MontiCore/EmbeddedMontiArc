<#-- (c) https://github.com/MontiCore/monticore -->
<#include "DdpgAgentParams.ftl">
<#if (config.policyNoise)??>
        'policy_noise': ${config.policyNoise},
</#if>
<#if (config.noiseClip)??>
        'noise_clip': ${config.noiseClip},
</#if>
<#if (config.policyDelay)??>
        'policy_delay': ${config.policyDelay},
</#if>
