/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { LanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { OCLLanguageWorkerContribution } from "./ocl-contribution";

export default new ContainerModule(bind => {
    bind(LanguageWorkerContribution).to(OCLLanguageWorkerContribution).inSingletonScope();
});
