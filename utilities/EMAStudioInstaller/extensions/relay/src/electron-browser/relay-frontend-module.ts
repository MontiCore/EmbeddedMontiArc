/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { RelayService } from "./relay-service";

export default new ContainerModule(bind => {
    bind(RelayService).toSelf().inSingletonScope();
});
