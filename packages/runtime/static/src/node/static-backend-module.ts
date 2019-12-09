/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core/lib/common";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { StaticPaths } from "../common";
import { StaticServerImpl } from "./static-server-impl";
import { StaticService, StaticServiceImpl } from "./static-service";

export default new ContainerModule(bind => {
    bind(StaticServiceImpl).toSelf().inSingletonScope();
    bind(StaticService).toService(StaticServiceImpl);
    bind(BackendApplicationContribution).toService(StaticServiceImpl);

    bind(StaticServerImpl).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(
            StaticPaths.PATH,
            () => ctx.container.get(StaticServerImpl)
        )
    ).inSingletonScope();
});
