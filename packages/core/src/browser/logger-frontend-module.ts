/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ILoggerServer } from "@theia/core/lib/common/logger-protocol";
import { LoggerBrowser } from "./browser-logger";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(ILoggerServer).to(LoggerBrowser).inSingletonScope();
});
