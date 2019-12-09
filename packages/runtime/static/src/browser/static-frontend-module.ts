/*
 * (c) https://github.com/MontiCore/monticore
 */
import { WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { StaticPaths, StaticServer } from "../common";

export default new ContainerModule(bind => {
    bind(StaticServer).toDynamicValue(
        ctx => ctx.container.get(WebSocketConnectionProvider).createProxy(StaticPaths.PATH)
    ).inSingletonScope();
});
