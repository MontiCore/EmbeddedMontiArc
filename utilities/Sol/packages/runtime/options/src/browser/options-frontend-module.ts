/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider } from "@theia/core";
import { FrontendApplicationContribution, WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { ValidatorPaths, ValidatorRegistry, ValidatorServer } from "../common";
import { bindCommon } from "../common/options-common-module";
import { FrontendValidatorRegistry } from "./frontend-validator-registry";
import { OptionFactory, OptionManager, OptionManagerImpl } from "./option-manager";
import { ListOption, ListOptionProps } from "./list-option";
import { PathOption, PathProps } from "./path-option";
import { StringOption, StringOptionProps } from "./string-option";
import { ValidatorService, ValidatorServiceImpl } from "./validator-service";

import "../../src/browser/style/options.css";

export default new ContainerModule(bind => {
    bindCommon(bind);
    bindContributionProvider(bind, OptionFactory);

    bind(OptionManagerImpl).toSelf().inSingletonScope();
    bind(OptionManager).toService(OptionManagerImpl);

    bind(FrontendValidatorRegistry).toSelf().inSingletonScope();
    bind(ValidatorRegistry).toService(FrontendValidatorRegistry);
    bind(FrontendApplicationContribution).toService(FrontendValidatorRegistry);

    bind(ValidatorServiceImpl).toSelf().inSingletonScope();
    bind(ValidatorService).toService(ValidatorServiceImpl);

    bind(ValidatorServer).toDynamicValue(
        ctx => ctx.container.get(WebSocketConnectionProvider).createProxy(ValidatorPaths.PATH)
    ).inSingletonScope();

    bind(OptionFactory).toDynamicValue(ctx => ({
        type: PathOption.TYPE,
        renderComponent: (props: PathProps) =>
            PathOption.createComponent(props, ctx.container)
    })).inSingletonScope();

    bind(OptionFactory).toDynamicValue(() => ({
        type: StringOption.TYPE,
        renderComponent: (props: StringOptionProps) =>
            StringOption.createComponent(props)
    })).inSingletonScope();

    bind(OptionFactory).toDynamicValue(ctx => ({
        type: ListOption.TYPE,
        renderComponent: (props: ListOptionProps) =>
            ListOption.createComponent(props, ctx.container)
    })).inSingletonScope();
});
