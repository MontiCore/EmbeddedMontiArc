/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import {
    ConnectionStatusService,
    ConnectionStatus
} from "@theia/core/lib/browser/connection-status-service";
import {
    FrontendApplicationContribution,
    FrontendApplication,
    DefaultFrontendApplicationContribution
} from "@theia/core/lib/browser/frontend-application";
import { MaybePromise } from "@theia/core/lib/common";
import { Event } from "@theia/core/lib/common/event";

@injectable()
export class BrowserFrontendConnectionStatusService implements ConnectionStatusService, FrontendApplicationContribution {
    public readonly currentState: ConnectionStatus;
    public readonly onStatusChange: Event<ConnectionStatus>;

    public onStart(app: FrontendApplication): MaybePromise<void> {
        return undefined;
    }

    public onStop(app: FrontendApplication): void {
        // NOOP
    }
}

@injectable()
export class BrowserApplicationConnectionStatusContribution extends DefaultFrontendApplicationContribution {}
