/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { ContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { memo } from "helpful-decorators";
import { inject, injectable, named } from "inversify";
import { Application } from "express";

/**
 * An interface representing an argument to be passed to either the docker run command or to the image.
 */
export interface DockerRunArgument {
    /**
     * Indicates whether the argument should be appended to docker run or to the arguments of the image.
     */
    for: "run" | "image",

    /**
     * A method which will be invoked once the docker container will be started and which resolves the argument to its
     * string value for the command.
     * @param hostPath The host path on which the docker run command will be executed.
     * @return A promise holding the resolved argument as string.
     */
    resolve: (hostPath: string) => Promise<string>;
}

export const DockerRunContribution = Symbol("DockerRunContribution");
/**
 * An interface to be implemented by classes which register [[DockerRunArgument]]s to be added to the docker run
 * command.
 */
export interface DockerRunContribution {
    /**
     * A method which can be used to register [[DockerRunArgument]]s.
     * @param registry The registry to which the [[DockerRunArgument]]s will be added.
     */
    registerArguments(registry: DockerRunRegistry): void;
}

export const DockerRunRegistry = Symbol("DockerRunRegistry");
/**
 * An interface to be implemented by classes which implement the necessary functionality to add and fetch registered
 * [[DockerRunArgument]]s.
 */
export interface DockerRunRegistry {
    /**
     * Fetches the registered [[DockerRunArgument]]s.
     * @return The registered [[DockerRunArgument]]s.
     */
    getArguments(): DockerRunArgument[];

    /**
     * Registers the given [[DockerRunArgument]].
     * @param argument The [[DockerRunArgument]] to be registered.
     */
    registerArgument(argument: DockerRunArgument): void;
}

@injectable()
export class DockerRunRegistryImpl implements DockerRunRegistry, ApplicationContribution {
    @inject(ContributionProvider) @named(DockerRunContribution)
    protected readonly provider: ContributionProvider<DockerRunContribution>;

    protected readonly arguments: Set<DockerRunArgument>;

    public constructor() {
        this.arguments = new Set();
    }

    @memo()
    protected getContributions(): DockerRunContribution[] {
        return this.provider.getContributions();
    }

    @memo()
    public getArguments(): DockerRunArgument[] {
        return Array.from(this.arguments.values());
    }

    public registerArgument(argument: DockerRunArgument): void {
        this.arguments.add(argument);
    }

    public async onConfigure(application: Application): Promise<void> {
        this.getContributions().forEach(contribution => contribution.registerArguments(this));
    }
}
