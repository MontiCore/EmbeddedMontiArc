/*
 * (c) https://github.com/MontiCore/monticore
 */
import { IPCConnectionProvider } from "@embeddedmontiarc/sol-external-core/lib/common";
import { CommandContribution } from "@theia/core";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { GUI_WINDOW_PATH, GUIWindowServer } from "../common/window";
import { GUIWindowCommandContribution, GUIWindowContribution } from "./window";

export default new ContainerModule(bind => {
    bind(GUIWindowCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toService(GUIWindowCommandContribution);

    bind(GUIWindowContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toService(GUIWindowContribution);

    bind(GUIWindowServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(GUI_WINDOW_PATH)
    ).inSingletonScope();
});
