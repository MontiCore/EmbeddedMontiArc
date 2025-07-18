/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FrontendApplicationContribution, FrontendApplication } from "@theia/core/lib/browser";
import { AutoPilotDistributedCommandContribution } from "./autopilot-distributed-command";
import { AutoPilotModelicaCommandContribution } from "./autopilot-modelica-command";
import { AutoPilotExecutingHandler } from "./autopilot-executing-handler";
import { WorkspaceService } from "@theia/workspace/lib/browser";

@injectable()
export class AutoPilotFrontendContribution implements FrontendApplicationContribution {
    @inject(AutoPilotDistributedCommandContribution)
    protected readonly distributed: AutoPilotDistributedCommandContribution;

    @inject(AutoPilotModelicaCommandContribution)
    protected readonly modelica: AutoPilotModelicaCommandContribution;

    @inject(AutoPilotExecutingHandler)
    protected readonly executingHandler: AutoPilotExecutingHandler;

    @inject(WorkspaceService)
    protected readonly workspaceService: WorkspaceService;

    public onStart(application: FrontendApplication): void {
        window.addEventListener("beforeunload", this.onWindowBeforeUnload.bind(this));
    }

    protected async onWindowBeforeUnload(event: BeforeUnloadEvent): Promise<void> {
        if (this.distributed.isRunning || this.modelica.isRunning || this.executingHandler.isRunning) {
            const close = !!this.workspaceService.workspace;

            event.preventDefault();

            event.returnValue = '';

            if (this.distributed.isRunning) await this.distributed.executeKill();
            if (this.modelica.isRunning) await this.modelica.executeKill();
            if (this.executingHandler.isRunning) await this.executingHandler.executeKill();

            if (close) window.close();
            else window.location.reload(true);
        }
    }
}
