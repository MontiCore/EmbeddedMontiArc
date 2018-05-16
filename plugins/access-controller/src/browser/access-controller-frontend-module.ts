/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { AccessContribution, AccessController } from "./access-controller";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import {
    FileSystemAccessContribution,
    EditorManagerAccessContribution,
    URIFactoryAccessContribution, WorkspaceAccessContribution
} from "./access-contributions";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, AccessContribution);

    bind(AccessController).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(AccessController)
    ).inSingletonScope();

    bind(FileSystemAccessContribution).toSelf().inSingletonScope();
    bind(AccessContribution).toDynamicValue(
        ctx => ctx.container.get(FileSystemAccessContribution)
    ).inSingletonScope();

    bind(EditorManagerAccessContribution).toSelf().inSingletonScope();
    bind(AccessContribution).toDynamicValue(
        ctx => ctx.container.get(EditorManagerAccessContribution)
    ).inSingletonScope();

    bind(URIFactoryAccessContribution).toSelf().inSingletonScope();
    bind(AccessContribution).toDynamicValue(
        ctx => ctx.container.get(URIFactoryAccessContribution)
    ).inSingletonScope();

    bind(WorkspaceAccessContribution).toSelf().inSingletonScope();
    bind(AccessContribution).toDynamicValue(
        ctx => ctx.container.get(WorkspaceAccessContribution)
    ).inSingletonScope();
});
