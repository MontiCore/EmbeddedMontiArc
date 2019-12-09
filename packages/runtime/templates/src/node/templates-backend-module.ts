/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider, ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { TemplatesPaths, TemplatesServer } from "../common";
import { TemplatesContribution, TemplatesRegistry, TemplatesRegistryImpl } from "./templates-registry";
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
