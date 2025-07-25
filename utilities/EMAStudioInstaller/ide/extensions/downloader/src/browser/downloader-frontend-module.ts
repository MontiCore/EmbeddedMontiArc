/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { DemosDownloader } from "./demos-downloader";
import { ZIPDownloader } from "./zip-downloader";

export default new ContainerModule(bind => {
    bind(DemosDownloader).toSelf().inSingletonScope();
    bind(ZIPDownloader).toSelf().inSingletonScope();
});
