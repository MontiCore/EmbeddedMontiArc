/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { TestConfigurationRunner } from "./test-configuration-runner";

import {
    ConfigurationRunnerContribution,
    ConfigurationRunnerRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/common";

@injectable()
export class TestConfigurationRunnerContribution implements ConfigurationRunnerContribution {
    public registerRunners(registry: ConfigurationRunnerRegistry): void {
        registry.registerRunner(TestConfigurationRunner);
    }
}
