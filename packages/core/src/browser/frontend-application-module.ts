/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import {
    FrontendConnectionStatusService,
    ApplicationConnectionStatusContribution,
    ConnectionStatusService
} from "@theia/core/lib/browser/connection-status-service";
import { FrontendApplicationContribution } from "@theia/core/lib/browser/frontend-application";
import {
    BrowserFrontendConnectionStatusService,
    BrowserApplicationConnectionStatusContribution
} from "./connection-status-service";
import { ApplicationServer } from "@theia/core/lib/common/application-protocol";
import { BrowserApplicationServer } from "../common";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind<ConnectionStatusService>(
        FrontendConnectionStatusService
    ).to(BrowserFrontendConnectionStatusService).inSingletonScope();
    rebind<FrontendApplicationContribution>(
        FrontendConnectionStatusService
    ).to(BrowserFrontendConnectionStatusService).inSingletonScope();
    rebind<FrontendApplicationContribution>(
        ApplicationConnectionStatusContribution
    ).to(BrowserApplicationConnectionStatusContribution).inSingletonScope();
    rebind(
        ApplicationServer
    ).to(BrowserApplicationServer).inSingletonScope();
});
