/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ValidatorContribution, ValidatorRegistry } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { injectable } from "inversify";
import { TestConfigurationValidator } from "./test-configuration-validator";

@injectable()
export class TestConfigurationValidatorContribution implements ValidatorContribution {
    public registerValidators(registry: ValidatorRegistry): void {
        registry.registerValidator(TestConfigurationValidator);
    }
}
