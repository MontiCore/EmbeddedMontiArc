/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { bindContributionProvider, ContributionProvider } from "@theia/core/lib/common";
import { ContainerModule, injectable, inject, named } from "inversify";
import { ConnectionHandler } from "@theia/core/lib/common";
import { WorkerApplicationContribution } from "../worker-application";
import { createWorkerConnection } from "./connection";

export const messagingWorkerModule = new ContainerModule(bind => {
    bind<WorkerApplicationContribution>(WorkerApplicationContribution).to(MessagingContribution);
    bindContributionProvider(bind, ConnectionHandler);
});

@injectable()
export class MessagingContribution implements WorkerApplicationContribution {
    public constructor(
        @inject(ContributionProvider)
        @named(ConnectionHandler)
        protected readonly handlers: ContributionProvider<ConnectionHandler>
    ) {}

    public onStart(worker: Worker): void {
        for (const handler of this.handlers.getContributions()) {
            const path = handler.path;

            try {
                createWorkerConnection({ path }, connection => handler.onConnection(connection));
            } catch (error) {
                console.error(error);
            }
        }
    }
}
