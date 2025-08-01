/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, named } from "inversify";
import { WorkerApplicationContribution } from "@elysium/core/lib/worker";
import { LanguageWorker, AnalyzeItem, OutlineItem } from "../common";
import { ContributionProvider } from "@theia/core/lib/common";
import { LanguageWorkerContribution } from "./language-worker-contribution";

@injectable()
export class LanguageWorkerImpl implements WorkerApplicationContribution, LanguageWorker {
    @inject(ContributionProvider) @named(LanguageWorkerContribution)
    protected readonly provider: ContributionProvider<LanguageWorkerContribution>;

    protected readonly map: Map<string, LanguageWorkerContribution> = new Map();

    public onStart(worker: Worker): void {
        for (const contribution of this.provider.getContributions()) {
            this.map.set(contribution.id, contribution);
        }
    }

    public async parse(id: string, docValue: string): Promise<void> {
        const worker = this.map.get(id);

        if (worker && await worker.isReady) worker.parse(docValue);
    }

    public async analyze(id: string): Promise<AnalyzeItem[]> {
        const worker = this.map.get(id);

        if (worker && await worker.isReady) return worker.analyze();
        else return [];
    }

    public async outline(id: string): Promise<OutlineItem[]> {
        const worker = this.map.get(id);

        if (worker && await worker.isReady) return worker.outline();
        else return [];
    }
}
