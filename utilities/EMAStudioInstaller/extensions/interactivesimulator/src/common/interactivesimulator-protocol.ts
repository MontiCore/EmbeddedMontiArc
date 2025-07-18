/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export namespace InteractiveSimulatorScripts {
    export const DEBUG: string = "debug" + (isWindows ? ".bat" : ".sh");
    export const DEBUG_WOSVG: string = "debug.wosvg" + (isWindows ? ".bat" : ".sh");
}

export namespace InteractiveSimulatorPaths {
    export const INTERACTIVESIMULATOR: string = "interactivesimulator";
}

export namespace InteractiveSimulatorStaticPaths {
    export const INTERACTIVESIMULATOR: string = "/interactivesimulator";
}
