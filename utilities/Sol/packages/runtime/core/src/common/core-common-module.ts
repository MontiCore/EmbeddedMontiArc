/*
 * (c) https://github.com/MontiCore/monticore
 */
import { interfaces } from "inversify";
import { ContextContainer } from "./context-container";

import Bind = interfaces.Bind;

export function bindCommon(bind: Bind) {
    bind(ContextContainer).toDynamicValue(ctx => ctx.container); // FIXME: Anti-Pattern, fix when Inversify allows unbinding named bindings.
}
