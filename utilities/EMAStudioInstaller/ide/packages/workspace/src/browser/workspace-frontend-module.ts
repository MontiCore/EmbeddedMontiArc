/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { BrowserWorkspaceContribution } from "./browser-workspace-contribution";
import { CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { WorkspaceServer } from "@theia/workspace/lib/common";
import { BrowserWorkspace } from "./browser-workspace";
import { WorkspaceFrontendContribution } from "@theia/workspace/lib/browser";
import { BrowserWorkspaceFrontendContribution } from "./workspace-frontend-contribution";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(BrowserWorkspaceContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(BrowserWorkspaceContribution)
    ).inSingletonScope();

    rebind(WorkspaceServer).to(BrowserWorkspace);

    bind(BrowserWorkspaceFrontendContribution).toSelf().inSingletonScope();
    rebind(WorkspaceFrontendContribution).to(BrowserWorkspaceFrontendContribution).inSingletonScope();
    rebind<MenuContribution>(WorkspaceFrontendContribution).toDynamicValue(
        ctx => ctx.container.get(BrowserWorkspaceFrontendContribution)
    );
    rebind<CommandContribution>(WorkspaceFrontendContribution).toDynamicValue(
        ctx => ctx.container.get(BrowserWorkspaceFrontendContribution)
    );
});
