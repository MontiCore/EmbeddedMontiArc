/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { WindowService } from "@theia/core/lib/browser/window/window-service";
import { DefaultExtendedWindowService, ExtendedWindowService } from "./window-service";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(ExtendedWindowService).to(DefaultExtendedWindowService).inSingletonScope();

    rebind(WindowService).to(DefaultExtendedWindowService).inSingletonScope();
});
