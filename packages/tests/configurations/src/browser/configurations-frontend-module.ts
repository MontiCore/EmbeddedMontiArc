/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConfigurationRunnerContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { ContainerModule } from "inversify";
import { TestConfigurationRunner } from "./test-configuration-runner";
import { TestConfigurationRunnerContribution } from "./test-configuration-runner-contribution";
import { TestConfigurationTypeContribution } from "./test-configuration-type-contribution";
import { ConfigurationTypeContribution } from "@embeddedmontiarc/sol-runtime-configurations/lib/browser";

export default new ContainerModule(bind => {
    bind(ConfigurationTypeContribution).to(TestConfigurationTypeContribution).inSingletonScope();

    bind(TestConfigurationRunner).toSelf().inSingletonScope();
    bind(ConfigurationRunnerContribution).to(TestConfigurationRunnerContribution).inSingletonScope();
});
