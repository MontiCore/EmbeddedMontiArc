/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MenuContribution } from "@theia/core";
import { ContainerModule } from "inversify";
import { bindCommon } from "../common/core-common-module";
import { CoreMenuContribution } from "./core-menu-contribution";

export default new ContainerModule(bind => {
    bindCommon(bind);

    bind(CoreMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toService(CoreMenuContribution);
});
