/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FrontendApplication, FrontendApplicationContribution } from "@theia/core/lib/browser/frontend-application";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { CLUSTERING_PATH, RUN_SCRIPT } from "../common";
import { RelayService, RelayMessage } from "@emastudio/relay/lib/electron-browser/relay-service";

export type ClusterFiddleMessageType = "Cluster" | "Clustered";

export interface ClusterFiddleMessage extends RelayMessage {
    type: ClusterFiddleMessageType;
}

@injectable()
export class ClusteringFrontendContribution implements FrontendApplicationContribution {
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(RelayService) protected readonly relayService: RelayService;

    public onStart(application: FrontendApplication): void {
        this.relayService.onMessage(this.onMessage.bind(this));
    }

    // tslint:disable-next-line:no-any
    protected async onMessage(message: any): Promise<void> {
        const data = <ClusterFiddleMessage>message;

        if (data.path === CLUSTERING_PATH) return this.handleMessage(data);
    }

    protected async handleMessage(data: ClusterFiddleMessage): Promise<void> {
        if (data.type === "Cluster") return this.handleClusterMessage();
    }

    protected async handleClusterMessage(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Cluster",
            plugin: "executing",
            script: RUN_SCRIPT
        });

        if (process) process.onExit(this.onProcessExit.bind(this));
    }

    protected onProcessExit(): void {
        this.relayService.sendMessage({ path: CLUSTERING_PATH, type: "Clustered" });
    }
}
