/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const VISUALIZE_SCRIPT: string = "visualize" + (isWindows ? ".bat" : ".sh");

export const VISUALIZATION_PATH_ID: string = "visualization";
export const VISUALIZATION_STATIC_PATH: string = "/visualization";
