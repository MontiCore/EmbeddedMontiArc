/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { StaticBackendContribution } from "./static-backend-contribution";

export default new ContainerModule(bind => {
    bind(StaticBackendContribution).toSelf().inSingletonScope();
    bind(BackendApplicationContribution).toService(StaticBackendContribution);
});
