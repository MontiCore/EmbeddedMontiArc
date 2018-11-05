/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { Container, ContainerModule } from "inversify";
import { PROCESS_PATH, ProcessServer } from "../common/process-protocol";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser/messaging";
import { Process, ProcessFactory, ProcessIdentifier } from "./process";
import { ProcessService } from "./process-service";
import { ProcessManager } from "./process-manager";
import { bindViewContribution, FrontendApplicationContribution, WidgetFactory } from "@theia/core/lib/browser";
import { ProcessWidget } from "./process-widget";
import { ProcessViewContribution } from "./process-view-contribution";
import { ProcessWidgetState } from "./process-widget-state";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(ProcessServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        return provider.createProxy<ProcessServer>(PROCESS_PATH);
    }).inSingletonScope();

    bind(Process).toSelf().inTransientScope();
    bind(ProcessFactory).toFactory(ctx =>
        (identifier: ProcessIdentifier) => {
            const child = new Container({ defaultScope: "Singleton" });

            child.parent = ctx.container;

            child.bind(ProcessIdentifier).toConstantValue(identifier);
            return child.get(Process);
        }
    );

    bind(ProcessService).toSelf().inSingletonScope();

    bind(ProcessManager).toSelf().inSingletonScope();

    bind(ProcessWidgetState).toSelf().inSingletonScope();

    bind(ProcessWidget).toSelf();
    bind(WidgetFactory).toDynamicValue(
        ctx => ({
            id: ProcessWidget.WIDGET_ID,
            createWidget: () => ctx.container.get<ProcessWidget>(ProcessWidget)
        })
    );

    bindViewContribution(bind, ProcessViewContribution);
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(ProcessViewContribution)
    ).inSingletonScope();
});
