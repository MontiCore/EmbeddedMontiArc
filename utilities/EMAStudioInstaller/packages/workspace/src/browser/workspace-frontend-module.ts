/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { workspacePath, WorkspaceServer } from "@theia/workspace/lib/common";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { EMAStudioWorkspaceFrontendContribution } from "./emastudio-workspace-frontend-contribution";
import { BrowserWorkspaceFrontendContribution } from "@elysium/workspace/lib/browser/workspace-frontend-contribution";
import { WorkspaceService as BaseWorkspaceService } from "@theia/workspace/lib/browser";
import { WorkspaceService } from "./workspace-service";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(WorkspaceServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        return provider.createProxy<WorkspaceServer>(workspacePath);
    }).inSingletonScope();

    rebind(BrowserWorkspaceFrontendContribution).to(EMAStudioWorkspaceFrontendContribution).inSingletonScope();

    rebind(BaseWorkspaceService).to(WorkspaceService).inSingletonScope();
});
