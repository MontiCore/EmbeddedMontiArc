/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { SELogoContribution } from "./se-logo-contribution";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(FrontendApplicationContribution).to(SELogoContribution).inSingletonScope();
});
