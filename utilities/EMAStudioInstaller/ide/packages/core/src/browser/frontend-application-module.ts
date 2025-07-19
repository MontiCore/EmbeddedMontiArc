/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import {
    FrontendConnectionStatusService
} from "@theia/core/lib/browser/connection-status-service";
import { ApplicationServer } from "@theia/core/lib/common/application-protocol";
import { BrowserApplicationServer } from "../common";
import { EnvVariablesServer } from "@theia/core/lib/common/env-variables";
import { BrowserEnvVariablesServer } from "./env-variables/env-variables-server";
import { BrowserFrontendConnectionStatusService } from "./connection-status-service";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(FrontendConnectionStatusService).to(BrowserFrontendConnectionStatusService).inSingletonScope();

    rebind(ApplicationServer).to(BrowserApplicationServer).inSingletonScope();

    rebind(EnvVariablesServer).to(BrowserEnvVariablesServer).inSingletonScope();
});
