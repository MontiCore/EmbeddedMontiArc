/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const REPORT_SCRIPT: string = "report" + (isWindows ? ".bat" : ".sh");
export const REPORT_STREAMS_SCRIPT: string = "report.streams" + (isWindows ? ".bat" : ".sh");

export const REPORTING_PATH_ID: string = "reporting";
export const REPORTING_STATIC_PATH: string = "/reporting";
