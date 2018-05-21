/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, named, postConstruct } from "inversify";
import { ContributionProvider } from "@theia/core/lib/common";

export const WorkerApplicationContribution = Symbol("WorkerApplicationContribution");

export interface WorkerApplicationContribution {
    /**
     * Initializes the `WorkerApplicationContribution`.
     */
    initialize?(): void;

    /**
     * Configures the `WorkerApplicationContribution`.
     */
    configure?(worker: Worker): void;

    /**
     * Triggers when the `WorkerApplication` is being started.
     */
    onStart?(worker: Worker): void;
}

@injectable()
export class WorkerApplication {
    @inject(ContributionProvider) @named(WorkerApplicationContribution)
    protected readonly provider: ContributionProvider<WorkerApplicationContribution>;

    @postConstruct()
    protected init(): void {
        this.initialize();
        this.configure();
    }

    protected initialize(): void {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.initialize) this.doInitialize(contribution);
        }
    }

    protected doInitialize(contribution: WorkerApplicationContribution): void {
        try {
            contribution.initialize!();
        } catch (error) {
            console.error('' + error);
        }
    }

    protected configure(): void {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.configure) this.doConfigure(contribution);
        }
    }

    protected doConfigure(contribution: WorkerApplicationContribution): void {
        try {
            // tslint:disable-next-line:no-any
            contribution.configure!(self as any);
        } catch (error) {
            console.error('' + error);
        }
    }

    public async start(): Promise<void> {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.onStart) this.doStart(contribution);
        }
    }

    protected doStart(contribution: WorkerApplicationContribution): void {
        try {
            // tslint:disable-next-line:no-any
            contribution.onStart!(self as any);
        } catch (error) {
            console.error('' + error);
        }
    }
}
