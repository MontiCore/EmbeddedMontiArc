/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ValidatorContribution, ValidatorRegistry } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable } from "inversify";
import { TestModuleValidator } from "./test-module-validator";

@injectable()
export class TestModuleValidatorContribution implements ValidatorContribution {
    public registerValidators(registry: ValidatorRegistry): void {
        registry.registerValidator(TestModuleValidator);
    }
}
