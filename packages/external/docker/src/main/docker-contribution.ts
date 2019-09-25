/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { PreparationContribution, PreparationRegistry } from "@embeddedmontiarc/sol-external-preparation/lib/main";
// import { MessageType } from "@theia/core/lib/common/message-service-protocol";
import { inject, injectable } from "inversify";
import { DockerRunService } from "./docker-run-service";
import { DockerMachineService } from "./docker-machine-service";

@injectable()
export class DockerContribution implements ApplicationContribution, PreparationContribution {
    @inject(DockerMachineService) protected readonly machine: DockerMachineService;
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(DockerRunService) protected readonly runner: DockerRunService;

    protected status: string;

    public async onStop(): Promise<void> {
        await this.killContainer();
        return this.resetMachineStatus();
    }

    public registerPreparations(registry: PreparationRegistry): void {
        registry.registerPreparation({
            id: "docker.machine.status",
            priority: 50790,
            prepare: () => this.configure()
        });
    }

    protected async configure(): Promise<void> {
        this.status = await this.machine.command("status");
    }

    protected async resetMachineStatus(): Promise<void> {
        /*if (this.status === "Stopped") {
            await this.notifications.showMessage("Shutting down Docker machine.", MessageType.Warning);
            await this.machine.command("stop");
        }*/
    }

    protected async killContainer(): Promise<void> {
        // await this.ide.kill();
    }
}
