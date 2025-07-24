/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { ZIPDownloader } from "./zip-downloader";
import { DownloadConfig } from "./downloader";

const USERNAME = "EmbeddedMontiArc";
const REPONAME = "Demos";
const BRANCHNAME = "master";

/**
 * Configuration for the DemosDownloader.
 */
export interface DemosDownloadConfig extends DownloadConfig {
    /**
     * Specifies which demo to download.
     */
    readonly demo: string;
}

@injectable()
export class DemosDownloader extends ZIPDownloader {
    public async download(config: DemosDownloadConfig): Promise<void> {
        const downloadConfig = this.toDownloadConfig(config);

        return super.download(downloadConfig);
    }

    protected toDownloadConfig(config: DemosDownloadConfig): DownloadConfig {
        return <DownloadConfig>{
            localURI: config.localURI,
            remoteURI: `https://raw.githubusercontent.com/${USERNAME}/${REPONAME}/${BRANCHNAME}/${config.demo}.zip`
        };
    }
}
