/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindBackendService } from "@embeddedmontiarc/sol-external-core/lib/main/messaging";
import { DockerPortContribution } from "@embeddedmontiarc/sol-external-docker/lib/main";
import { DockerRunContribution } from "@embeddedmontiarc/sol-external-docker/lib/main/docker-run-registry";
import { ContainerModule } from "inversify";
import { GUI_WINDOW_PATH, GUIWindowServer } from "../common/window";
import { GUIProcessPortContribution } from "./gui-process-port-contribution";
import { GUIProcessRunContribution } from "./gui-process-run-contribution";
import { GUIWindowServerImpl } from "./window";

export default new ContainerModule(bind => {
    bind(GUIProcessPortContribution).toSelf().inSingletonScope();
    bind(DockerPortContribution).toService(GUIProcessPortContribution);

    bind(GUIWindowServerImpl).toSelf().inSingletonScope();
    bind(GUIWindowServer).toService(GUIWindowServerImpl);

    bind(GUIProcessRunContribution).toSelf().inSingletonScope();
    bind(DockerRunContribution).toService(GUIProcessRunContribution);

    bindBackendService(bind, GUI_WINDOW_PATH, GUIWindowServer);
});
