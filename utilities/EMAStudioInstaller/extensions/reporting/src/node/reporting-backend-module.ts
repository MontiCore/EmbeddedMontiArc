/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { ReportingPathContribution } from "./reporting-path-contribution";
import { ReportingStaticContribution } from "./reporting-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(ReportingPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(ReportingPathContribution)
    ).inSingletonScope();

    bind(ReportingStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(ReportingStaticContribution)
    ).inSingletonScope();
});
