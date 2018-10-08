/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { MiniBrowserService } from "@theia/mini-browser/lib/common/mini-browser-service";

@injectable()
export class BrowserMiniBrowserService implements MiniBrowserService {
    public async supportedFileExtensions(): Promise<Readonly<{ extension: string, priority: number }>[]> {
        return [];
    }
}
