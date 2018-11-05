/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const TESTING_PATH_ID: string = "testing";

export const TESTING_SCRIPT: string = "test" + (isWindows ? ".bat" : ".sh");
export const TESTING_ALL_SCRIPT: string = "test.all" + (isWindows ? ".bat" : ".sh");
