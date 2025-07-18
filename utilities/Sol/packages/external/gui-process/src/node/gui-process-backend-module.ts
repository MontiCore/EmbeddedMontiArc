/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution, CliContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { GUIProcessCLIContribution } from "./gui-process-cli-contribution";
import { GUIProcessContribution } from "./gui-process-contribution";

export default new ContainerModule(bind => {
    bind(GUIProcessContribution).toSelf().inSingletonScope();
    bind(BackendApplicationContribution).toService(GUIProcessContribution);

    bind(GUIProcessCLIContribution).toSelf().inSingletonScope();
    bind(CliContribution).toService(GUIProcessCLIContribution);
});
