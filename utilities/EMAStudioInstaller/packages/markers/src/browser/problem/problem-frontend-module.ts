/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ProblemManager as BaseProblemManager } from "@theia/markers/lib/browser";
import { ProblemManager } from "./problem-manager";

export default new ContainerModule((bind, isBound, unbind, rebind) => {
    rebind(BaseProblemManager).to(ProblemManager).inSingletonScope();
});
