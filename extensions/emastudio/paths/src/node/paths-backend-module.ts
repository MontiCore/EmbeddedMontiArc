/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathsRegistry, PathContribution } from "./paths-registry";
import { bindContributionProvider } from "@theia/core/lib/common";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { RootPathContribution } from "./path-contributions";
import { PathsServerImpl } from "./paths-server";
import { PATHS_PATH, PathsServer } from "../common";
import { ConnectionHandler } from "@theia/core/lib/common/messaging";
import { JsonRpcConnectionHandler } from "@theia/core/lib/common/messaging";
import { CliContribution } from "@theia/core/lib/node";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, PathContribution);

    bind(PathsRegistry).toSelf().inSingletonScope();
    bind(BackendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(PathsRegistry)
    ).inSingletonScope();

    bind(RootPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(RootPathContribution)
    ).inSingletonScope();
    bind(CliContribution).toDynamicValue(
        ctx => ctx.container.get(RootPathContribution)
    ).inSingletonScope();

    bind(PathsServerImpl).toSelf().inSingletonScope();
    bind(PathsServer).toService(PathsServerImpl);
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(PATHS_PATH, () =>
            ctx.container.get(PathsServer)
        )
    ).inSingletonScope();
});
