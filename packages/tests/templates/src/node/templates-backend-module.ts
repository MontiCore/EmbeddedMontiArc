/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { TemplatesContribution } from "@embeddedmontiarc/sol-runtime-templates/lib/node/templates-registry";
import { ContainerModule } from "inversify";
import { TestTemplateContribution } from "./test-template-contribution";
import { TestTemplateValidator } from "./test-template-validator";
import { TestTemplateValidatorContribution } from "./test-template-validator-contribution";

export default new ContainerModule(bind => {
    bind(TestTemplateValidator).toSelf().inSingletonScope();

    bind(TestTemplateValidatorContribution).toSelf().inSingletonScope();
    bind(ValidatorContribution).toService(TestTemplateValidatorContribution);

    bind(TestTemplateContribution).toSelf().inSingletonScope();
    bind(TemplatesContribution).toService(TestTemplateContribution);
});
