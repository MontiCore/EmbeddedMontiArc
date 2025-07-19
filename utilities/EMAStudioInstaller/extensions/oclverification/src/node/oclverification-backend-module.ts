/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { OCLVerificationPathContribution } from "./oclverification-path-contribution";
import { OCLVerificationStaticContribution } from "./oclverification-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";

export default new ContainerModule(bind => {
    bind(OCLVerificationPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(OCLVerificationPathContribution)
    ).inSingletonScope();

    bind(OCLVerificationStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(OCLVerificationStaticContribution)
    ).inSingletonScope();
});
