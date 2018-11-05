/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, postConstruct } from "inversify";
import { FrontendConnectionStatusService } from "@theia/core/lib/browser/connection-status-service";
import { DefaultFrontendApplicationContribution } from "@theia/core/lib/browser/frontend-application";

@injectable()
export class BrowserFrontendConnectionStatusService extends FrontendConnectionStatusService {
    @postConstruct()
    protected init(): void {
        // NOOP
    }
}

@injectable()
export class BrowserApplicationConnectionStatusContribution extends DefaultFrontendApplicationContribution {}
