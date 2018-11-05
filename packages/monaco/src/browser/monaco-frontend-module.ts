/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { MonacoDiagnosticsContribution } from "./monaco-diagnostics-contribution";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { MonacoOutlineContribution as BaseMonacoOutlineContribution } from "@theia/monaco/lib/browser/monaco-outline-contribution";
import { MonacoOutlineContribution } from "./monaco-outline-contribution";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(MonacoOutlineContribution).toSelf().inSingletonScope();
    rebind<FrontendApplicationContribution>(
        BaseMonacoOutlineContribution
    ).toDynamicValue(ctx => ctx.container.get(MonacoOutlineContribution));

    bind(MonacoDiagnosticsContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(ctx => ctx.container.get(MonacoDiagnosticsContribution));
});
