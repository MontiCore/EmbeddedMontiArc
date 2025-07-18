/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Container, interfaces } from "inversify";
import { Tool } from "./tool";
import { ToolFactory } from "./tool-factory";

import ServiceIdentifier = interfaces.ServiceIdentifier;
import Bind = interfaces.Bind;
import Newable = interfaces.Newable;

export function bindToolFactory(bind: Bind, factory: ServiceIdentifier<ToolFactory>, tool: Newable<Tool>) {
    bind(factory).toFactory(ctx => (argstring: string, uuid: string) => {
        const container = new Container({ defaultScope: "Singleton" });

        container.parent = ctx.container;

        container.bind(tool).toSelf();
        return container.get(tool).run(argstring, uuid);
    });
}
