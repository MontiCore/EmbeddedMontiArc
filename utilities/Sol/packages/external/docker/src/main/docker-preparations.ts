/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BinaryService } from "@embeddedmontiarc/sol-external-core/lib/main/binary";
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { PreparationContribution, PreparationRegistry } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { MessageType } from "@theia/core/lib/common/message-service-protocol";
import { Deferred } from "@theia/core/lib/common/promise-util";
import { spawn } from "child_process";
import { inject, injectable } from "inversify";
import { DockerMachineService } from "./docker-machine-service";

@injectable()
export class DockerPreparations implements PreparationContribution { // TODO: Add docker pull of image.
    @inject(BinaryService) protected readonly binaries: BinaryService;
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(DockerMachineService) protected readonly machine: DockerMachineService;

    public registerPreparations(registry: PreparationRegistry): void {
        registry.registerPreparation({
            id: "docker.binary",
            priority: 50900,
            prepare: () => this.checkDockerBinary()
        });

        registry.registerPreparation({
            id: "docker.machine.binary",
            priority: 50800,
            prepare: () => this.checkDockerMachineBinary()
        });

        registry.registerPreparation({
            id: "docker.machine.start",
            priority: 50700,
            prepare: () => this.startDockerMachine()
        });

        registry.registerPreparation({
            id: "docker.daemon",
            priority: 50600,
            prepare: () => this.checkDockerDaemon()
        });
    }

    protected async checkDockerBinary(): Promise<void> {
        await this.notifications.showMessage("Checking whether docker binary is available.");

        if (await this.binaries.has("docker")) return this.notifications.showMessage("docker binary is available.");
        else throw new Error(`docker is either not installed or not known to PATH environment variable.`);
    }

    protected async checkDockerMachineBinary(): Promise<void> {
        await this.notifications.showMessage("Checking whether docker-machine binary is available.");

        if (await this.binaries.has("docker-machine")) return this.notifications.showMessage("docker-machine binary is available.");
        else throw new Error(`docker-machine is either not installed or not known to PATH environment variable.`);
    }

    protected async startDockerMachine(): Promise<void> {
        await this.notifications.showMessage("Checking whether Docker machine is currently running.");

        const status = await this.machine.command("status");

        if (status === "Running") {
            return this.notifications.showMessage("Docker machine is currently running.");
        } else {
            await this.notifications.showMessage("Starting Docker machine...", MessageType.Warning);
            await this.machine.command("start");
            return this.notifications.showMessage("Docker machine has been started.");
        }
    }

    protected async checkDockerDaemon(): Promise<void> {
        await this.notifications.showMessage("Checking whether Docker daemon is currently running.");

        const deferred = new Deferred<void>();
        const process = spawn("docker", ["--help"]);
        const error = new Error(`Docker daemon has not been started. Please make sure that the Docker service is started.`);
        const timer = setTimeout(() => deferred.reject(error), 3000);

        process.once("exit", deferred.resolve);

        await deferred.promise;

        clearTimeout(timer);
        return this.notifications.showMessage("Docker daemon is currently running.");
    }
}
