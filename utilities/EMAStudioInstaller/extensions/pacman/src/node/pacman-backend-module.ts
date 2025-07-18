/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { PacManPathContribution } from "./pacman-path-contribution";
import { PacManStaticContribution } from "./pacman-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(PacManPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(PacManPathContribution)
    ).inSingletonScope();

    bind(PacManStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(PacManStaticContribution)
    ).inSingletonScope();
});
