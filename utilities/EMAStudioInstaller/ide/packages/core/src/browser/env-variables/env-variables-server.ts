/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { EnvVariablesServer, EnvVariable } from "@theia/core/lib/common/env-variables";

@injectable()
export class BrowserEnvVariablesServer implements EnvVariablesServer {
    public async getValue(key: string): Promise<EnvVariable | undefined> {
        return undefined;
    }

    public async getVariables(): Promise<EnvVariable[]> {
        return [];
    }
}
