/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ModuleCreatorContribution } from "@embeddedmontiarc/sol-runtime-modules/lib/node/creator/module-creator-registry";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { ContainerModule } from "inversify";
import { TestModuleCreator } from "./test-module-creator";
import { TestModuleCreatorContribution } from "./test-module-creator-contribution";
import { TestModuleValidator } from "./test-module-validator";
import { TestModuleValidatorContribution } from "./test-module-validator-contribution";

export default new ContainerModule(bind => {
    bind(TestModuleCreator).toSelf().inSingletonScope();

    bind(TestModuleCreatorContribution).toSelf().inSingletonScope();
    bind(ModuleCreatorContribution).toService(TestModuleCreatorContribution);

    bind(TestModuleValidator).toSelf().inSingletonScope();

    bind(TestModuleValidatorContribution).toSelf().inSingletonScope();
    bind(ValidatorContribution).toService(TestModuleValidatorContribution);
});
