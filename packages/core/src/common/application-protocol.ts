/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { ApplicationServer, ApplicationInfo, ExtensionInfo } from "@theia/core/lib/common/application-protocol";

@injectable()
export class BrowserApplicationServer implements ApplicationServer {
    public async getApplicationInfo(): Promise<ApplicationInfo | undefined> {
        return { name: "Elysium IDE", version: "0.1.0" };
    }

    public async getExtensionsInfos(): Promise<ExtensionInfo[]> {
        return [];
    }
}
