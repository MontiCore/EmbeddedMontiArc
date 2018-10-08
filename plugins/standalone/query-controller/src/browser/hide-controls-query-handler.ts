/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { QueryHandler } from "./query-controller";
import { ApplicationShell } from "@theia/core/lib/browser/shell";
import URI from "@elysium/core/lib/common/uri";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";
import { WorkspaceInitiatorStateService } from "@elysium/workspace-initiator/lib/browser";

/**
 * `QueryHandler` which handles the collapsing of the side-panels.
 */
@injectable()
export class HideControlsQueryHandler implements QueryHandler {
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(FrontendApplicationStateService) protected readonly appStateService: FrontendApplicationStateService;
    @inject(WorkspaceInitiatorStateService) protected readonly stateService: WorkspaceInitiatorStateService;

    public canHandle(uri: URI): number {
        return uri.hasQueryParam("hideControls") ? 100 : 0;
    }

    public isChainable(): boolean {
        return true;
    }

    public async handle(uri: URI): Promise<void> {
        const panelsOpened = this.stateService.getPanelOpened();

        if (panelsOpened) {
            const hideControls = uri.getQueryParam("hideControls");
            const bits = hideControls!.split(',');

            await this.appStateService.reachedState("ready");

            return this.handleBits(bits);
        } else {
            return this.stateService.setPanelOpened(true);
        }
    }

    protected async handleBits(bits: string[]): Promise<void> {
        if (+bits[0]) this.shell.collapsePanel("left");
        if (+bits[1]) this.shell.collapsePanel("right");
    }
}
