/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { VisualizationPathContribution } from "./visualization-path-contribution";
import { VisualizationStaticContribution } from "./visualization-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(VisualizationPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(VisualizationPathContribution)
    ).inSingletonScope();

    bind(VisualizationStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(VisualizationStaticContribution)
    ).inSingletonScope();
});
