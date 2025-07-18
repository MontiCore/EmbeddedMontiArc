/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import * as BrowserFS from "browserfs";
import { FileSystemConfiguration } from "browserfs";
import { FSModule } from "browserfs/dist/node/core/FS";

export async function configure(config: FileSystemConfiguration): Promise<FSModule> {
    return new Promise<FSModule>((resolve, reject) => {
        // tslint:disable-next-line: no-any
        const System = {} as any;

        BrowserFS.install(System);
        BrowserFS.configure(config, error => {
            const fs = System.require("fs");

            if (error) reject(error);
            else resolve(fs);
        });
    });
}
