/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { MessageClient } from "@theia/core/lib/common";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    rebind(MessageClient).to(MessageClient).inSingletonScope();
});
