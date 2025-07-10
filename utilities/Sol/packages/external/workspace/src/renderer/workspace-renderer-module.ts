/*
 * (c) https://github.com/MontiCore/monticore
 */
import { IPCConnectionProvider } from "@embeddedmontiarc/sol-external-core/lib/common";
import { RouteContribution } from "@embeddedmontiarc/sol-external-core/lib/renderer/router-fragment";
import { ContainerModule } from "inversify";
import { WORKSPACE_PATH, WorkspaceServer } from "../common";
import { Workspaces } from "./workspaces";

export default new ContainerModule(bind => {
    bind(RouteContribution).toConstantValue({
        path: "/workspaces",
        component: Workspaces
    });

    bind(WorkspaceServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(WORKSPACE_PATH)
    ).inSingletonScope();
});
