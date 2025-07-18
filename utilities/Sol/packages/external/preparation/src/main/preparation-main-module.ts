/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution, ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { bindContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { ContainerModule } from "inversify";
import { PreparationPhase } from "./preparation-phase";
import { PreparationContribution, PreparationRegistry, PreparationRegistryImpl } from "./preparation-registry";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, PreparationContribution);

    bind(PreparationRegistryImpl).toSelf().inSingletonScope();
    bind(PreparationRegistry).toService(PreparationRegistryImpl);
    bind(ApplicationContribution).toService(PreparationRegistryImpl);

    bind(PreparationPhase).toSelf().inSingletonScope();
    bind(ApplicationPhase).toService(PreparationPhase);
});
