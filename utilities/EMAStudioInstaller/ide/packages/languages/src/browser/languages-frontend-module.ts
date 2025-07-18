/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { LanguageWorker, languageWorkerPath } from "../common";
import { WorkerConnectionProvider } from "@elysium/core/lib/browser";
import { Cloud9Converter } from "./cloud9-converter";

export default new ContainerModule((bind, unbind) => {
    bind(Cloud9Converter).toSelf().inSingletonScope();
    bind(LanguageWorker).toDynamicValue(ctx => {
        const provider = ctx.container.get(WorkerConnectionProvider);
        return provider.createProxy<LanguageWorker>(languageWorkerPath);
    }).inSingletonScope();
});
