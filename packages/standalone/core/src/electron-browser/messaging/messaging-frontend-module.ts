/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { WorkerConnectionProvider } from "../../browser/messaging/connection";

export default new ContainerModule(bind => {
    bind(WorkerConnectionProvider).toSelf().inSingletonScope();
});
