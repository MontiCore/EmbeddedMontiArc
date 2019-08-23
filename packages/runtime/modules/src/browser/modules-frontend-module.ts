/* (c) https://github.com/MontiCore/monticore */
import { CommandContribution, MenuContribution } from "@theia/core";
import { ContainerModule } from "inversify";
import { ModulesCommandContribution } from "./modules-command-contribution";
import { ModulesMenuContribution } from "./modules-menu-contribution";

export default new ContainerModule(bind => {
    bind(CommandContribution).to(ModulesCommandContribution).inSingletonScope();
    bind(MenuContribution).to(ModulesMenuContribution).inSingletonScope();
});
