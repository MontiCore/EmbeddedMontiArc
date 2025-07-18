/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider, JsonRpcConnectionHandler } from "@theia/core/lib/common";
import { ConnectionHandler } from "@theia/core/lib/common/messaging/handler";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { MODULE_PATH } from "../common";
import { ModuleCreatorContribution } from "./creator";
import { ModuleCreatorDelegator, ModuleCreatorDelegatorImpl } from "./creator";
import { ModuleCreatorRegistry, ModuleCreatorRegistryImpl } from "./creator";
import { ModuleServerImpl } from "./module-server-impl";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ModuleCreatorContribution);

    bind(ModuleCreatorDelegatorImpl).toSelf().inSingletonScope();
    bind(ModuleCreatorDelegator).toService(ModuleCreatorDelegatorImpl);

    bind(ModuleCreatorRegistryImpl).toSelf().inSingletonScope();
    bind(ModuleCreatorRegistry).toService(ModuleCreatorRegistryImpl);
    bind(BackendApplicationContribution).toService(ModuleCreatorRegistryImpl);

    bind(ModuleServerImpl).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(
            MODULE_PATH,
            () => ctx.container.get(ModuleServerImpl)
        )
    ).inSingletonScope();
});
