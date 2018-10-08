/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution } from "@emastudio/paths/lib/node";
import { ScriptsPathContribution } from "./scripts-path-contribution";

export default new ContainerModule(bind => {
    bind(ScriptsPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(ScriptsPathContribution)
    ).inSingletonScope();
});
