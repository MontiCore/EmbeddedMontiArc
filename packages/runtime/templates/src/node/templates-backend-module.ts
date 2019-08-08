/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
import { bindContributionProvider, ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { TemplatesPaths, TemplatesServer } from "../common/templates-protocol";
import { TemplatesContribution, TemplatesRegistry, TemplatesRegistryImpl } from "./templates-contribution";
import { TemplatesServerImpl } from "./templates-server-impl";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, TemplatesContribution);

    bind(TemplatesRegistryImpl).toSelf().inSingletonScope();
    bind(TemplatesRegistry).toService(TemplatesRegistryImpl);
    bind(BackendApplicationContribution).toService(TemplatesRegistryImpl);

    bind(TemplatesServerImpl).toSelf().inSingletonScope();
    bind(TemplatesServer).toService(TemplatesServerImpl);
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(TemplatesPaths.PATH, () =>
            ctx.container.get(TemplatesServer)
        )
    ).inSingletonScope();
});
