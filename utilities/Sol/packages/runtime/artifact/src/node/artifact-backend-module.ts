/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider } from "@theia/core";
import { EnvVariablesServer } from "@theia/core/lib/common/env-variables";
import { ContainerModule } from "inversify";
import { EnvVariablesContribution, EnvVariablesRegistry, EnvVariablesServerImpl } from "./env-variables-server";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bindContributionProvider(bind, EnvVariablesContribution);

    bind(EnvVariablesServerImpl).toSelf().inSingletonScope();
    rebind(EnvVariablesServer).toService(EnvVariablesServerImpl);
    bind(EnvVariablesRegistry).toService(EnvVariablesServerImpl);
});
