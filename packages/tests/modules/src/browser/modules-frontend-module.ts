/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ModuleTypeContribution } from "@embeddedmontiarc/sol-runtime-modules/lib/browser/module-type-registry";
import { ContainerModule } from "inversify";
import { TestModuleTypeContribution } from "./test-module-type-contribution";

export default new ContainerModule(bind => {
    bind(TestModuleTypeContribution).toSelf().inSingletonScope();
    bind(ModuleTypeContribution).toService(TestModuleTypeContribution);
});
