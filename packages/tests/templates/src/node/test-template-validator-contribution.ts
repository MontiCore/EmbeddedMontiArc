/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ValidatorContribution, ValidatorRegistry } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable } from "inversify";
import { TestTemplateValidator } from "./test-template-validator";

@injectable()
export class TestTemplateValidatorContribution implements ValidatorContribution {
    public registerValidators(registry: ValidatorRegistry): void {
        registry.registerValidator(TestTemplateValidator);
    }
}
