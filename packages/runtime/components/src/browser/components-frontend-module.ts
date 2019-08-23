/* (c) https://github.com/MontiCore/monticore */
import { bindContributionProvider } from "@theia/core";
import { ContainerModule } from "inversify";
import { ComponentFactory, ComponentManager, ComponentManagerImpl } from "./component-manager";
import { ListComponent, ListComponentProps } from "./list-component";
import { PathComponent, PathComponentProps } from "./path-component";
import { StringComponent, StringComponentProps } from "./string-component";
import { RefObject } from "react";

import "../../src/browser/style/components.css";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ComponentFactory);

    bind(ComponentManager).to(ComponentManagerImpl).inSingletonScope();

    bind(ComponentFactory).toDynamicValue(ctx => ({
        type: PathComponent.TYPE,
        renderComponent: (ref: RefObject<PathComponent>, props: PathComponentProps) =>
            PathComponent.createComponent(ref, props, ctx.container)
    })).inSingletonScope();

    bind(ComponentFactory).toDynamicValue(() => ({
        type: StringComponent.TYPE,
        renderComponent: (ref: RefObject<StringComponent>, props: StringComponentProps) =>
            StringComponent.createComponent(ref, props)
    })).inSingletonScope();

    bind(ComponentFactory).toDynamicValue(ctx => ({
        type: ListComponent.TYPE,
        renderComponent: (ref: RefObject<ListComponent>, props: ListComponentProps) =>
            ListComponent.createComponent(ref, props, ctx.container)
    })).inSingletonScope();
});
