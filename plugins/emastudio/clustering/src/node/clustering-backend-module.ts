/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PathContribution} from "@emastudio/paths/lib/node";
import { ClusteringPathContribution } from "./clustering-path-contribution";
import { ClusteringStaticContribution } from "./clustering-static-contribution";
import { StaticContribution } from "@emastudio/static/lib/node";
import { ClusteringBackendContribution } from "./clustering-backend-contribution";
import { BackendApplicationContribution } from "@theia/core/lib/node";

export default new ContainerModule(bind => {
    bind(ClusteringPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(ClusteringPathContribution)
    ).inSingletonScope();

    bind(ClusteringStaticContribution).toSelf().inSingletonScope();
    bind(StaticContribution).toDynamicValue(
        ctx => ctx.container.get(ClusteringStaticContribution)
    ).inSingletonScope();

    bind(ClusteringBackendContribution).toSelf().inSingletonScope();
    bind(BackendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(ClusteringBackendContribution)
    ).inSingletonScope();
});
