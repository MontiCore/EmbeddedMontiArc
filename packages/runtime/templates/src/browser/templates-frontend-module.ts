/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommandContribution, MenuContribution } from "@theia/core";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { Container, ContainerModule } from "inversify";
import { TemplatesPaths, TemplatesServer } from "../common";
import { TemplateDialog, TemplateDialogFactory, TemplateDialogProps } from "./template-dialog";
import { TemplatesCommandContribution } from "./templates-command-contribution";
import { TemplatesMenuContribution } from "./templates-menu-contribution";

export default new ContainerModule(bind => {
    bind(TemplatesCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).to(TemplatesCommandContribution).inSingletonScope();

    bind(TemplatesMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).to(TemplatesMenuContribution).inSingletonScope();

    bind(TemplateDialogFactory).toFactory(ctx => (props: TemplateDialogProps) => {
        const container = new Container();

        container.parent = ctx.container;

        container.bind(TemplateDialogProps).toConstantValue(props);
        container.bind(TemplateDialog).toSelf();
        return container.get(TemplateDialog);
    });

    bind(TemplatesServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        return provider.createProxy<TemplatesServer>(TemplatesPaths.PATH);
    }).inSingletonScope();
});
