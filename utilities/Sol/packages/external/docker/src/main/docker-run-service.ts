/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Application } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { WindowService } from "@embeddedmontiarc/sol-external-core/lib/main/window/window-service";
import { PreparationContribution, PreparationRegistry } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { bind } from "helpful-decorators";
import { inject, injectable } from "inversify";
import { Readable } from "stream";
import { DockerRunArgument, DockerRunRegistry } from "./docker-run-registry";
import { DockerService } from "./docker-service";

import * as EventEmitter from "eventemitter3";

export const DockerRunService = Symbol("DockerRunService");
/**
 * An interface to be implemented by classes which implement the necessary functionality to interact with the IDE
 * Docker container.
 */
export interface DockerRunService {
    /**
     * Checks whether the IDE Docker container is currently running.
     * @return A promise holding a boolean which indicates whether the IDE Docker container is currently running or not.
     */
    isRunning(): Promise<boolean>;

    /**
     * Starts the IDE Docker container with a given path as workspace.
     * @param hostPath The host path which will be mounted into the Docker container and which acts as workspace.
     */
    run(hostPath: string): Promise<void>;

    /**
     * Kills the currently running IDE Docker container if currently running.
     */
    kill(): Promise<void>;

    /**
     * A method which can be used to register an event handler for a given event.
     * @param event The event to which the event handler should be registered.
     * @param handler The event handler to be registered.
     */
    on(event: "run", handler: () => void): void;
    on(event: "exit", handler: () => void): void;
}

@injectable()
export class DockerRunServiceImpl extends EventEmitter implements DockerRunService, PreparationContribution {
    @inject(DockerRunRegistry) protected readonly registry: DockerRunRegistry;
    @inject(Application) protected readonly application: Application;
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(DockerService) protected readonly docker: DockerService;
    @inject(WindowService) protected readonly windows: WindowService;

    public async isRunning(): Promise<boolean> {
        const name = this.application.getName();
        const output = await this.docker.command(`ps --format "{{ json . }}" --filter name=${name}`);

        return output.length > 0;
    }

    /*
     * TODO: If there should ever be a cross-platform way of getting the IP of the host interface,
     *  the IDE should actively notify the main process of its ready state instead of analyzing
     *  the output.
     */
    public async run(hostPath: string): Promise<void> {
        await this.kill();

        const command = await this.buildCommand(hostPath);
        const process = this.docker.commandAsProcess(command);

        console.debug(command);

        process.on("exit", this.onProcessExit);
        process.stdout!.on("data", data => console.debug(data.toString()));
        process.stderr!.on("data", data => console.debug(data.toString()));

        if (process && process.stdout) await this.doRun(process.stdout);
        else await Promise.reject(`Could not run IDE.`);
    }

    protected async buildCommand(hostPath: string): Promise<string> {
        const image = this.application.getImage();
        const name = this.application.getName();

        const args = this.registry.getArguments();
        const runArgs = await this.buildArguments(hostPath, args.filter(arg => arg.for === "run"));
        const imageArgs = await this.buildArguments(hostPath, args.filter(arg => arg.for === "image"));

        return `run --rm --name ${name} ${runArgs} ${image} ${imageArgs}`;
    }

    protected async buildArguments(hostPath: string, args: DockerRunArgument[]): Promise<string> {
        const result = [];

        for (const arg of args) {
            result.push(await arg.resolve(hostPath));
        }

        return result.join(' ');
    }

    protected doRun(stdout: Readable): Promise<void> {
        return new Promise((resolve, reject) => {
            const timer = setTimeout(reject, 30000);
            const onData = (data: any)  => { // tslint:disable-line:no-any
                if (data.toString().indexOf("Theia app listening on") > 1) {
                    clearTimeout(timer);
                    stdout.off("data", onData);
                    this.emit("run");
                    resolve();
                }
            };

            stdout.on("data", onData);
        });
    }

    public async kill(): Promise<void> {
        if (await this.isRunning()) {
            const name = this.application.getName();

            await this.notifications.showMessage("Shutting down currently running IDE instance...");
            await this.docker.command(`container kill ${name}`);
            await this.clean();
            await this.notifications.showMessage("IDE instance has been shut down.");
        }
    }

    protected async clean(): Promise<void> {
        if (await this.isRunning()) {
            const name = this.application.getName();

            await this.notifications.showMessage("Cleaning remains of IDE instance...");
            await this.docker.command(`container rm ${name}`);
            return this.notifications.showMessage("Remains of IDE instance have been cleaned.");
        }
    }

    public registerPreparations(registry: PreparationRegistry): void {
        registry.registerPreparation({
            id: "docker.run.kill",
            priority: 50400,
            prepare: () => this.kill()
        });
    }

    @bind
    protected onProcessExit(): void {
        this.emit("exit");
    }
}
