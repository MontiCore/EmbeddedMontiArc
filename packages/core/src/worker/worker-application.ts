/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, named } from "inversify";
import { ContributionProvider } from "@theia/core/lib/common";

export const WorkerApplicationContribution = Symbol("WorkerApplicationContribution");

export interface WorkerApplicationContribution {
    initialize?(): void;
    configure?(worker: Worker): void;
    onStart?(worker: Worker): void;
}

@injectable()
export class WorkerApplication {
    public constructor(
        @inject(ContributionProvider) @named(WorkerApplicationContribution)
        protected readonly provider: ContributionProvider<WorkerApplicationContribution>
    ) {
        this.initialize();
        this.configure();
    }

    protected initialize(): void {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.initialize) {
                try {
                    contribution.initialize();
                } catch (error) {
                    console.error('' + error);
                }
            }
        }
    }

    protected configure(): void {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.configure) {
                try {
                    // tslint:disable-next-line:no-any
                    contribution.configure(self as any);
                } catch (err) {
                    console.error(err.toString());
                }
            }
        }
    }

    public async start(): Promise<void> {
        for (const contribution of this.provider.getContributions()) {
            if (contribution.onStart) {
                try {
                    // tslint:disable-next-line:no-any
                    contribution.onStart(self as any);
                } catch (error) {
                    console.error('' + error);
                }
            }
        }
    }
}
