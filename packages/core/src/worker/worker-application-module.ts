/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { WorkerApplication, WorkerApplicationContribution } from "./worker-application";

export default new ContainerModule(bind => {
    bind(WorkerApplication).toSelf().inSingletonScope();
    bindContributionProvider(bind, WorkerApplicationContribution);
});
