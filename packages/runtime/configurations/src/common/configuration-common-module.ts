/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider } from "@theia/core";
import { interfaces } from "inversify";
import { ConfigurationProcessorWatcher } from "./configuration-processor-watcher";
import { ConfigurationRunnerDelegator, ConfigurationRunnerDelegatorImpl } from "./configuration-runner-delegator";
import { ConfigurationRunnerContribution } from "./configuration-runner-registry";

import Bind = interfaces.Bind;

export function bindCommon(bind: Bind) {
    bindContributionProvider(bind, ConfigurationRunnerContribution);

    bind(ConfigurationRunnerDelegator).to(ConfigurationRunnerDelegatorImpl).inSingletonScope();
    bind(ConfigurationProcessorWatcher).toSelf().inSingletonScope();
}
