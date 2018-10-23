/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const CHECK_SCRIPT: string = "check.ocl" + (isWindows ? ".bat" : ".sh");
export const VISUALIZE_SCRIPT: string = "visualize.cd" + (isWindows ? ".bat" : ".sh");

export const OCLVERIFICATION_PATH_ID: string = "oclverification";
export const OCLVERIFICATION_STATIC_PATH: string = "/oclverification";
