/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { LanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { StreamUnitsLanguageWorkerContribution } from "./streamunits-contribution";

export default new ContainerModule(bind => {
    bind(LanguageWorkerContribution).to(StreamUnitsLanguageWorkerContribution).inSingletonScope();
});
