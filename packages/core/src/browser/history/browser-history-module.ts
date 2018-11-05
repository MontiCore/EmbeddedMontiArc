/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { HistoryService, DefaultHistoryService } from "./history-service";

export default new ContainerModule(bind => {
    bind(HistoryService).to(DefaultHistoryService).inSingletonScope();
});
