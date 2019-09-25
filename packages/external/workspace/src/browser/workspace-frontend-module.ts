/*
 * (c) https://github.com/MontiCore/monticore
 */
import { IPCConnectionProvider } from "@embeddedmontiarc/sol-external-core/lib/common";
import { FrontendApplication as FrontendApplicationBase, StorageService } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { WorkspaceFrontendContribution as BaseWorkspaceFrontendContribution } from "@theia/workspace/lib/browser";
import { WORKSPACE_PATH, WorkspaceServer } from "../common";
import { FrontendApplication } from "./frontend-application";
import { WorkspaceFrontendContribution } from "./workspace-frontend-contribution";
import { WorkspaceStorageService } from "./workspace-storage-service";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(BaseWorkspaceFrontendContribution).to(WorkspaceFrontendContribution).inSingletonScope();

    bind(WorkspaceServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(WORKSPACE_PATH)
    ).inSingletonScope();

    bind(WorkspaceStorageService).toSelf().inSingletonScope();
    rebind(StorageService).toService(WorkspaceStorageService);

    rebind(FrontendApplicationBase).to(FrontendApplication).inSingletonScope();
});
