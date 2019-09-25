/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { ContributionProvider } from "@theia/core";
import { memo } from "helpful-decorators";
import { inject, injectable, named } from "inversify";
import { Application } from "express";

export const DockerPortContribution = Symbol("DockerPortContribution");
/**
 * An interface to be implemented by classes which register ports to be bound during initiation of the IDE.
 */
export interface DockerPortContribution {
    /**
     * A method which can be used to register ports to be bound during instantiation of the IDE.
     * @param registry The registry to which the ports should be added.
     */
    registerPorts(registry: DockerPortRegistry): void;
}

@injectable()
export class DefaultDockerPortContribution implements DockerPortContribution {
    public registerPorts(registry: DockerPortRegistry): void {
        registry.registerPort(3000);
    }
}

export const DockerPortRegistry = Symbol("DockerPortRegistry");
/**
 * An interface to be implemented by a class which implements the functionality necessary to register, unregister,
 * and fetch the registered ports.
 */
export interface DockerPortRegistry {
    /**
     * Fetch the registered ports from the registry.
     * @return The registered ports.
     */
    getRegisteredPorts(): number[];

    /**
     * Register the given port.
     * @param port The port to be registered.
     */
    registerPort(port: number): void;

    /**
     * Unregister the given port.
     * @param port The port to be unregistered.
     */
    unregisterPort(port: number): void;
}

@injectable()
export class DockerPortRegistryImpl implements DockerPortRegistry, ApplicationContribution {
    @inject(ContributionProvider) @named(DockerPortContribution)
    protected readonly provider: ContributionProvider<DockerPortContribution>;

    protected readonly ports: Set<number>;

    public constructor() {
        this.ports = new Set();
    }

    @memo()
    public getContributions(): DockerPortContribution[] {
        return this.provider.getContributions();
    }

    public getRegisteredPorts(): number[] {
        return Array.from(this.ports);
    }

    public registerPort(port: number): void {
        if (this.ports.has(port)) throw new Error(`Port '${port}' has already been registered.`);
        else this.ports.add(port);
    }

    public unregisterPort(port: number): void {
        if (this.ports.has(port)) this.ports.delete(port);
        else throw new Error(`Port '${port}' has never been registered.`);
    }

    public async onConfigure(application: Application): Promise<void> {
        this.getContributions().forEach(contribution => contribution.registerPorts(this));
    }
}
