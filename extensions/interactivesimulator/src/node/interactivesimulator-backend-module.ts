/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { InteractiveSimulatorPathContribution } from "./interactivesimulator-path-contribution";
import { InteractiveSimulatorStaticContribution } from "./interactivesimulator-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(InteractiveSimulatorPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(InteractiveSimulatorPathContribution)
    ).inSingletonScope();

    bind(InteractiveSimulatorStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(InteractiveSimulatorStaticContribution)
    ).inSingletonScope();
});
