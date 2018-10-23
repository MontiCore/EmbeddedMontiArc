/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { TestingPathContribution } from "./testing-path-contribution";

export default new ContainerModule(bind => {
    bind(TestingPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(TestingPathContribution)
    ).inSingletonScope();
});
