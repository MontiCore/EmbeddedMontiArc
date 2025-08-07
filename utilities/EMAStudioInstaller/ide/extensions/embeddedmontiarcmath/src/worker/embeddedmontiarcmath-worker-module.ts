/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { LanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EmbeddedMontiArcMathLanguageWorkerContribution } from "./embeddedmontiarcmath-contribution";

export default new ContainerModule(bind => {
    bind(LanguageWorkerContribution).to(EmbeddedMontiArcMathLanguageWorkerContribution).inSingletonScope();
});
