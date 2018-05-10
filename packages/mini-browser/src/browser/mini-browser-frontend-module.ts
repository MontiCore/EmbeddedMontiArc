/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { MiniBrowserOpenHandler as BaseMiniBrowserOpenHandler } from "@theia/mini-browser/lib/browser/mini-browser-open-handler";
import { MiniBrowserOpenHandler } from "./mini-browser-open-handler";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(MiniBrowserOpenHandler).toSelf().inSingletonScope();
    rebind(BaseMiniBrowserOpenHandler).to(MiniBrowserOpenHandler).inSingletonScope();
});
