/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/**
 * Interface describing the configuration of a download process.
 */
export interface DownloadConfig {
    /**
     * The local path where the downloaded files will be stored.
     */
    readonly localURI: string;

    /**
     * The uri to the ZIP archive to be downloaded.
     */
    readonly remoteURI?: string;
}

export const Downloader = Symbol("Downloader");

/**
 * Interface which will be implemented by concrete downloaders.
 */
export interface Downloader {
    /**
     * Initiates the download process on the given configuration.
     */
    download(config: DownloadConfig): Promise<void>;
}
