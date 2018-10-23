/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const EMAM2WASM_SCRIPT: string = "emam2wasm" + (isWindows ? ".bat" : ".sh");
