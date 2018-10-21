/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const EXECUTE_KILL_SCRIPT: string = "execute.kill" + (isWindows ? ".bat" : ".sh");

export const EXECUTE_DISTRIBUTED_SCRIPT: string = "execute.distributed" + (isWindows ? ".bat" : ".sh");
export const EXECUTE_DISTRIBUTED_KILL_SCRIPT: string = "execute.distributed.kill" + (isWindows ? ".bat" : ".sh");

export const EXECUTE_MODELICA_SCRIPT: string = "execute.modelica" + (isWindows ? ".bat" : ".sh");
export const EXECUTE_MODELICA_KILL_SCRIPT: string = "execute.modelica.kill" + (isWindows ? ".bat" : ".sh");
