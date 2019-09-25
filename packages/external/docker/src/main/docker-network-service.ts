/*
 * (c) https://github.com/MontiCore/monticore
 */
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { PreparationContribution, PreparationRegistry } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { DockerMachineService } from "./docker-machine-service";
import { DockerPortRegistry } from "./docker-port-registry";
import { DockerRunContribution, DockerRunRegistry } from "./docker-run-registry";
import { DockerRunService } from "./docker-run-service";
import { DockerService } from "./docker-service";

import * as getPort from "get-port";

export const DockerNetworkService = Symbol("DockerNetworkService");
/**
 * An interface to be implemented by classes which operate on the network of Docker.
 */
export interface DockerNetworkService {
    /**
     * Fetches the IP of the Docker container.
     * @return The IP of the Docker container.
     */
    getInternalIP(): string | undefined;

    /**
     * Searches and "reserves" a free port on the host system.
     * @param internalPort The port in the Docker container for which the host port should be allocated.
     * @return The allocated port.
     */
    allocate(internalPort: number): Promise<number>;

    /**
     * Frees the previously allocated ports.
     */
    free(): void;

    /**
     * Resolves the host port from a given Docker container port.
     * @param internalPort The Docker container port for which the host port should be resolved.
     * @return The port allocated for the given Docker container port.
     */
    resolve(internalPort: number): number | undefined;
}

@injectable()
export class DockerNetworkServiceImpl implements DockerNetworkService, PreparationContribution, DockerRunContribution {
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(DockerService) protected readonly docker: DockerService;
    @inject(DockerMachineService) protected readonly machine: DockerMachineService;
    @inject(DockerPortRegistry) protected readonly registry: DockerPortRegistry;
    @inject(DockerRunService) protected readonly runner: DockerRunService;

    protected readonly mapping: Map<number, number>; // Key = Internal, Value = External

    protected internalIP: string | undefined;

    public constructor() {
        this.mapping = new Map();
    }

    @postConstruct()
    protected init(): void {
        this.runner.on("exit", this.onRunnerExit);
    }

    public getInternalIP(): string | undefined {
        return this.internalIP;
    }

    public async allocate(internalPort: number): Promise<number> {
        if (this.mapping.has(internalPort)) return this.mapping.get(internalPort)!;
        else return this.doAllocate(internalPort);
    }

    protected async doAllocate(internalPort: number): Promise<number> {
        let externalPort = await getPort();
        const externalPorts = Array.from(this.mapping.values());

        while (externalPorts.indexOf(externalPort) > -1) {
            externalPort = await getPort();
        }

        this.mapping.set(internalPort, externalPort);
        return externalPort;
    }

    public free(): void {
        this.mapping.clear();
    }

    public resolve(internalPort: number): number | undefined {
        return this.mapping.get(internalPort);
    }

    public registerPreparations(registry: PreparationRegistry): void {
        registry.registerPreparation({
            id: "docker.network.internal",
            priority: 50500,
            prepare: () => this.resolveInternalIP()
        });
    }

    protected async resolveInternalIP(): Promise<void> {
        await this.notifications.showMessage("Resolving internal IP...");

        this.internalIP = await this.machine.command("ip");

        return this.notifications.showMessage(`Internal IP is '${this.internalIP || "unknown"}'.`);
    }

    public registerArguments(registry: DockerRunRegistry): void {
        registry.registerArgument({
            for: "run",
            resolve: () => this.resolvePorts()
        });
    }

    protected async resolvePorts(): Promise<string> {
        const ports = [];
        const internalPorts = this.registry.getRegisteredPorts();
        const length = internalPorts.length;
        const externalPorts = await this.allocatePorts(internalPorts);

        for (let i = 0; i < length; i++) {
            ports.push(`-p ${externalPorts[i]}:${internalPorts[i]}`);
        }

        return ports.join(' ');
    }

    protected async allocatePorts(internalPorts: number[]): Promise<number[]> {
        const ports = [];
        const length = internalPorts.length;

        for (let i = 0; i < length; i++) {
            const port = await this.allocate(internalPorts[i]);

            ports.push(port);
        }

        return ports;
    }

    @bind
    protected onRunnerExit(): void {
        this.free();
    }
}
