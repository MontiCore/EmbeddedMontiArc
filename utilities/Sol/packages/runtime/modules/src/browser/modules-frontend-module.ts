/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution, WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { bindContributionProvider, CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { Container, ContainerModule } from "inversify";
import { MODULE_PATH, ModuleServer } from "../common";
import { ModuleCommandContribution } from "./module-command-contribution";
import { ModuleDialog, ModuleDialogFactory, ModuleDialogProps } from "./module-dialog";
import { ModuleMenuContribution } from "./module-menu-contribution";
import { ModuleTypeRegistry, ModuleTypeContribution, ModuleTypeRegistryImpl } from "./module-type-registry";

import "../../src/browser/style/modules.css";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ModuleTypeContribution);

    bind(ModuleMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toService(ModuleMenuContribution);

    bind(ModuleCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toService(ModuleCommandContribution);

    bind(ModuleTypeRegistryImpl).toSelf().inSingletonScope();
    bind(ModuleTypeRegistry).toService(ModuleTypeRegistryImpl);
    bind(FrontendApplicationContribution).toService(ModuleTypeRegistryImpl);

    bind(ModuleServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);

        return provider.createProxy(MODULE_PATH);
    }).inSingletonScope();

    bind(ModuleDialogFactory).toFactory(ctx => (props: ModuleDialogProps) => {
        const container = new Container();

        container.parent = ctx.container;

        container.bind(ModuleDialog).toSelf();
        container.bind(ModuleDialogProps).toConstantValue(props);
        return container.get(ModuleDialog);
    });
});
