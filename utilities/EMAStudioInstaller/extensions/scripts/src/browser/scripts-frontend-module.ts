/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ScriptsConditions } from "./scripts-conditions";
import { ScriptsService } from "./scripts-service";

export default new ContainerModule(bind => {
    bind(ScriptsConditions).toSelf().inSingletonScope();
    bind(ScriptsService).toSelf().inSingletonScope();
});
