/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { SuperMarioPathContribution } from "./supermario-path-contribution";
import { SuperMarioStaticContribution } from "./supermario-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(SuperMarioPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(SuperMarioPathContribution)
    ).inSingletonScope();

    bind(SuperMarioStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(SuperMarioStaticContribution)
    ).inSingletonScope();
});
