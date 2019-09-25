/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { bindBackendService } from "@embeddedmontiarc/sol-external-core/lib/main/messaging";
import { PreparationContribution } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { ContainerModule } from "inversify";
import { WORKSPACE_PATH, WorkspaceServer } from "../common";
import { WorkspacePhase } from "./workspace-phase";
import { WorkspaceServerImpl } from "./workspace-server-impl";

export default new ContainerModule(bind => {
    bind(WorkspacePhase).toSelf().inSingletonScope();
    bind(ApplicationPhase).toService(WorkspacePhase);

    bind(WorkspaceServerImpl).toSelf().inSingletonScope();
    bind(WorkspaceServer).toService(WorkspaceServerImpl);
    bind(PreparationContribution).toService(WorkspaceServerImpl);

    bindBackendService(bind, WORKSPACE_PATH, WorkspaceServer);
});
