/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution, ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { PreparationContribution } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { bindContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { ContainerModule } from "inversify";
import { DockerContribution } from "./docker-contribution";
import { DockerRunService, DockerRunServiceImpl } from "./docker-run-service";
import { DockerMachineService, DockerMachineServiceImpl } from "./docker-machine-service";
import { DockerNetworkService, DockerNetworkServiceImpl } from "./docker-network-service";
import { DockerPhase } from "./docker-phase";
import { DockerPreparations } from "./docker-preparations";
import { DockerRunContribution, DockerRunRegistry, DockerRunRegistryImpl } from "./docker-run-registry";
import { DockerService, DockerServiceImpl } from "./docker-service";

import {
    DefaultDockerPortContribution,
    DockerPortContribution,
    DockerPortRegistry,
    DockerPortRegistryImpl
} from "./docker-port-registry";
import { DockerVolumeService, DockerVolumeServiceImpl } from "./docker-volume-service";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, DockerPortContribution);
    bindContributionProvider(bind, DockerRunContribution);

    bind(DockerMachineServiceImpl).toSelf().inSingletonScope();
    bind(DockerMachineService).toService(DockerMachineServiceImpl);

    bind(DockerServiceImpl).toSelf().inSingletonScope();
    bind(DockerService).toService(DockerServiceImpl);

    bind(DockerPreparations).toSelf().inSingletonScope();
    bind(PreparationContribution).toService(DockerPreparations);

    bind(DockerContribution).toSelf().inSingletonScope();
    bind(ApplicationContribution).toService(DockerContribution);
    bind(PreparationContribution).toService(DockerContribution);

    bind(DockerNetworkServiceImpl).toSelf().inSingletonScope();
    bind(DockerNetworkService).toService(DockerNetworkServiceImpl);
    bind(PreparationContribution).toService(DockerNetworkServiceImpl);
    bind(DockerRunContribution).toService(DockerNetworkServiceImpl);

    bind(DockerPortRegistryImpl).toSelf().inSingletonScope();
    bind(DockerPortRegistry).toService(DockerPortRegistryImpl);
    bind(ApplicationContribution).toService(DockerPortRegistryImpl);

    bind(DefaultDockerPortContribution).toSelf().inSingletonScope();
    bind(DockerPortContribution).toService(DefaultDockerPortContribution);

    bind(DockerRunServiceImpl).toSelf().inSingletonScope();
    bind(DockerRunService).toService(DockerRunServiceImpl);
    bind(PreparationContribution).toService(DockerRunServiceImpl);

    bind(DockerPhase).toSelf().inSingletonScope();
    bind(ApplicationPhase).toService(DockerPhase);

    bind(DockerRunRegistryImpl).toSelf().inSingletonScope();
    bind(DockerRunRegistry).toService(DockerRunRegistryImpl);
    bind(ApplicationContribution).toService(DockerRunRegistryImpl);

    bind(DockerVolumeServiceImpl).toSelf().inSingletonScope();
    bind(DockerVolumeService).toService(DockerVolumeServiceImpl);
    bind(DockerRunContribution).toService(DockerVolumeServiceImpl);
});
