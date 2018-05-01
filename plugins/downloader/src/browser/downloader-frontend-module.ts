/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { DemosDownloader, DemosDownloaderImpl } from "./demos-downloader";

export default new ContainerModule(bind => {
    bind(DemosDownloaderImpl).toSelf().inSingletonScope();
    bind(DemosDownloader).toDynamicValue(ctx => ctx.container.get(DemosDownloaderImpl));
});
