/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { CommonFrontendContribution as BaseCommonFrontendContribution } from "@theia/core/lib/browser";
import { CommonFrontendContribution } from "./common-frontend-contribution";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(CommonFrontendContribution).toSelf().inSingletonScope();
    rebind(BaseCommonFrontendContribution).to(CommonFrontendContribution).inSingletonScope();
});
