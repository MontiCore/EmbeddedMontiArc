/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CommonConfigurationValidator } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { injectable } from "inversify";
import { TestConfigurationErrors, TestConfigurationOptions } from "../common";

@injectable()
export class TestConfigurationValidator
        extends CommonConfigurationValidator<TestConfigurationOptions, TestConfigurationErrors> {
    public constructor() {
        super("id");
    }

    public async validate(options: TestConfigurationOptions): Promise<TestConfigurationErrors> {
        return { name: undefined, path: undefined };
    }
}
