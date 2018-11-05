/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { LanguageWorkerImpl } from "./language-worker";
import { LanguageWorker, languageWorkerPath } from "../common";
import { ConnectionHandler, JsonRpcConnectionHandler, bindContributionProvider } from "@theia/core/lib/common";
import { LanguageWorkerContribution } from "./language-worker-contribution";
import { WorkerApplicationContribution } from "@elysium/core/lib/worker";

export default new ContainerModule(bind => {
    bind(LanguageWorkerImpl).toSelf().inSingletonScope();
    bind(WorkerApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(LanguageWorkerImpl)
    ).inSingletonScope();
    bind(LanguageWorker).toService(LanguageWorkerImpl);
    bind(ConnectionHandler).toDynamicValue(
        ctx => new JsonRpcConnectionHandler(
            languageWorkerPath, () => ctx.container.get(LanguageWorker)
        )
    ).inSingletonScope();
    bindContributionProvider(bind, LanguageWorkerContribution);
});
