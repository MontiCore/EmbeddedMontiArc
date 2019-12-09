/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { TestConfigurationCoordinator } from "./test-configuration-coordinator";

import {
    ConfigurationCoordinatorContribution,
    ConfigurationCoordinatorRegistry
} from "@embeddedmontiarc/sol-runtime-configurations/lib/node";

@injectable()
export class TestConfigurationCoordinatorContribution implements ConfigurationCoordinatorContribution {
    public registerCoordinators(registry: ConfigurationCoordinatorRegistry): void {
        registry.registerCoordinator(TestConfigurationCoordinator);
    }
}
