/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const EXECUTE_SCRIPT: string = "execute" + (isWindows ? ".bat" : ".sh");
