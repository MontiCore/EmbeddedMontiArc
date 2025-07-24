/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConfigurationRunnerContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { ConfigurationCoordinatorContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { ContainerModule } from "inversify";
import { TestConfigurationCoordinator } from "./test-configuration-coordinator";
import { TestConfigurationCoordinatorContribution } from "./test-configuration-coordinator-contribution";
import { TestConfigurationRunner } from "./test-configuration-runner";
import { TestConfigurationRunnerContribution } from "./test-configuration-runner-contribution";
import { TestConfigurationValidator } from "./test-configuration-validator";
import { TestConfigurationValidatorContribution } from "./test-configuration-validator-contribution";

export default new ContainerModule(bind => {
    bind(TestConfigurationValidator).toSelf().inSingletonScope();

    bind(ValidatorContribution).to(TestConfigurationValidatorContribution).inSingletonScope();

    bind(TestConfigurationCoordinator).toSelf().inSingletonScope();
    bind(ConfigurationCoordinatorContribution).to(TestConfigurationCoordinatorContribution).inSingletonScope();

    bind(TestConfigurationRunner).toSelf().inSingletonScope();
    bind(ConfigurationRunnerContribution).to(TestConfigurationRunnerContribution).inSingletonScope();
});
