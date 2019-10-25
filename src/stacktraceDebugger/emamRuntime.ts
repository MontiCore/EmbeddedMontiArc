/* (c) https://github.com/MontiCore/monticore */
/*---------------------------------------------------------
 * Copyright (C) Microsoft Corporation. All rights reserved.
 *--------------------------------------------------------*/

import { EventEmitter } from 'events';
import { emaStacktraces } from './stacktraceParser';
import { DebugProtocol } from 'vscode-debugprotocol';
import { existsSync, readFileSync } from 'fs'
import { TestCache } from './testCache';
import { EMATestRunner } from './testRunner';
import { window } from 'vscode';
import { RuntimeLogger } from './RuntimeLogger';
import * as log4js from 'log4js';

export interface EmamBreakpoint {
	id: number;
	line: number;
	verified: boolean;
}

export class EmamRuntime extends EventEmitter {
	// the initial (and one and only) file we are 'debugging'
	private testRunner: TestCache;
	private _sourceFile: string;
	private stacktraces: emaStacktraces;
	private modelbase: string;
	private lastManualIndex: number = -1;
	private logger: RuntimeLogger = new RuntimeLogger(this);
	private mavenPomPath: string;

	public setSourceFile(path: string) {
		this._sourceFile = path.replace("\/\/", "/");
	}

	public getVariables(): DebugProtocol.Variable[] {
		const variables = new Array<DebugProtocol.Variable>();
		for (var v of this.stacktraces[this.current_stack_index].vars) {
			variables.push({
				name: v.name,
				type: "unknown",
				value: v.value,
				variablesReference: 0
			});
		}
		return variables
	}



	public get sourceFile() {
		return this._sourceFile;
	}

	// the contents (= lines) of the one and only file
	private _sourceLines: string[];

	// This is the next line that will be 'executed'
	private current_stack_index = -1;

	// maps from sourceFile to array of Emam breakpoints
	private _breakPoints = new Map<string, EmamBreakpoint[]>();

	// since we want to send breakpoint events, we will assign an id to every event
	// so that the frontend can match events with breakpoints.
	private _breakpointId = 1;


	constructor(mavenPomPath: string) {
		super();
		this.mavenPomPath = mavenPomPath;
	}

	public debugLog(text: string) {
		this.logger.log(text);
	}

	/**
	 * Start executing the given program.
	 */
	public start(program: string, modelbase: string, stopOnEntry: boolean) {
		log4js.getLogger().trace("start");
		if (!existsSync(modelbase)) {
			window.showErrorMessage("Modelbase " + modelbase + " does not exist!");
			setTimeout(() => this.sendEvent('end'), 2000);
		} else {
			this.logger.log("Start debugging the Stream tests for " + program);
			const targetBasePath = modelbase + "/target";
			this.testRunner = new TestCache(new EMATestRunner(targetBasePath, this.mavenPomPath, modelbase, this.logger));
			this.modelbase = modelbase;
			this.logger.log("Parsing stacktraces...");
			const tmpStacktraces = this.testRunner.getStacktraces(program);
			if (tmpStacktraces) {
				this.stacktraces = tmpStacktraces;
				this.debugLog("Start debugging");
				if (stopOnEntry) {
					this.step();
				} else {
					this.continue();
				}
			} else {
				setTimeout(() => this.sendEvent('end'), 2000);
			}
		}
	}

	/**
	 * Continue execution to the end/beginning.
	 */
	public continue(reverse = false) {
		log4js.getLogger().trace("continue!");
		log4js.getLogger().trace("#breakpoints: " + this._breakPoints.size);
		this.run(reverse, undefined);
	}

	/**
	 * Step to the next/previous non empty line.
	 */
	public step(reverse = false, event = 'stopOnStep') {
		log4js.getLogger().trace("step");
		this.lastManualIndex = this.current_stack_index;
		this.run(reverse, event);
	}

	public stepIn() {
		this.run(false, "stepIn");
	}

	public stepOut() {
		log4js.getLogger().trace("stepOut");
		this.lastManualIndex = this.current_stack_index;
		this.run(false, "stepOut");
	}

	/**
	 * Returns a fake 'stacktrace' where every 'stackframe' is a word from the current line.
	 */
	public stack(startFrame: number, endFrame: number): any {
		const frames = new Array<any>();
		// every word of the current line becomes a stack frame.
		var i: number = 0;
		for (let f of this.stacktraces[this.current_stack_index]) {
			frames.push({
				index: i,
				name: f.instanceName,
				file: this.modelbase + "/" + f.fileName,
				line: f.line - 1,
				col: f.col,
				endCol: f.endCol
			});
			i++;
		}
		return {
			frames: frames,
			count: frames.length
		};
	}

	/*
	 * Set breakpoint in file with given line.
	 */
	public setBreakPoint(rawPath: string, line: number): EmamBreakpoint {
		log4js.getLogger().trace("setBreakPoint");
		let path = rawPath.replace("\/\/", "/");
		log4js.getLogger().debug("Set bp on " + path + ":" + line);
		const bp = <EmamBreakpoint>{ verified: false, line, id: this._breakpointId++ };
		let bps = this._breakPoints.get(path);
		if (!bps) {
			bps = new Array<EmamBreakpoint>();
			this._breakPoints.set(path, bps);
		}
		bps.push(bp);
		this.setSourceFile(path);
		this.verifyBreakpoints(path);

		return bp;
	}

	/*
	 * Clear breakpoint in file with given line.
	 */
	public clearBreakPoint(path: string, line: number): EmamBreakpoint | undefined {
		log4js.getLogger().trace("clearBreakPoint");
		let bps = this._breakPoints.get(path);
		if (bps) {
			const index = bps.findIndex(bp => bp.line === line);
			if (index >= 0) {
				const bp = bps[index];
				bps.splice(index, 1);
				return bp;
			}
		}
		return undefined;
	}

	public clearBreakpoints(path: string): void {
		log4js.getLogger().trace("clearBreakpoints");
		this._breakPoints.delete(path);
	}

	// private methods

	/**
	 * Run through the file.
	 * If stepEvent is specified only run a single step and emit the stepEvent.
	 */
	private run(reverse = false, stepEvent?: string) {
		log4js.getLogger().trace("run " + (stepEvent ? stepEvent : ""));
		if (reverse) {
			for (let ln = this.current_stack_index - 1; ln >= 0; ln--) {
				if (this.fireEventsForCurrentStacktrace(ln, stepEvent, reverse)) {
					this.current_stack_index = ln;
					return;
				}
			}
			// no more lines: stop at first line
			this.current_stack_index = 0;
			this.sendEvent('stopOnEntry');
		} else {
			for (let st = this.current_stack_index + 1; st < this.stacktraces.length; st++) {
				log4js.getLogger().trace("run for");
				this.setSourceFile(this.modelbase + "/" + this.stacktraces[st][0].fileName);
				if (this.fireEventsForCurrentStacktrace(st, stepEvent, reverse)) {
					this.current_stack_index = st;
					return true;
				}
			}
			// no more lines: run to end
			this.sendEvent('end');
		}
	}

	private verifyBreakpoints(path: string): void {
		log4js.getLogger().trace("Start ver for " + path);
		let prefix = "";
		if (path.startsWith("file:")) {
			prefix = "file:"
		}
		this._sourceLines = readFileSync(prefix + path, { "encoding": "utf-8", "flag": "r" }).split("\n");
		let bps = this._breakPoints.get(path);
		if (bps) {
			for (let bp of bps) {
				if (!bp.verified && bp.line < this._sourceLines.length) {
					if (this._sourceLines.length - 2 > bp.line) {
						bp.line = 0;
					} else {
						bp.line = this._sourceLines.length - 1;
					}

					bp.verified = true;
					this.sendEvent('breakpointValidated', bp);
				}
			}
		}
	}

	private fireEventsForCurrentStacktrace(ln: number, stepEvent?: string, reverse?: boolean): boolean {
		log4js.getLogger().trace("fireEventsForCurrentStacktrace");
		// is there a breakpoint?
		const breakpoints = this._breakPoints.get(this._sourceFile);
		if (breakpoints) {
			for (let bp of breakpoints) {
				const stLine = this.stacktraces[ln][0].line;
				const a = bp.line == stLine;
				const b = (stLine > 1 && bp.line >= stLine);
				if (a || b) {
					this.sendEvent('stopOnBreakpoint');
					return true;
				} else {
					log4js.getLogger().trace("Mismatch! bp.line: " + bp.line + ", stLine: " + stLine);
				}
			}
		} else {
			log4js.getLogger().trace("No breakpoints for " + this._sourceFile);
		}

		if (stepEvent) {
			if (stepEvent === "stepIn") {
				this.sendEvent("stopOnStep");
				return true;
			}

			if (stepEvent === "stepOut" && this.lastManualIndex !== -1) {
				let lmStacktrace = this.stacktraces[this.lastManualIndex];
				let curStacktrace = this.stacktraces[ln];

				if (curStacktrace.length === lmStacktrace.length && lmStacktrace[0].fileName !== curStacktrace[0].fileName) {
					this.sendEvent("stopOnStep");
					return true;
				}

				if (curStacktrace.length < lmStacktrace.length) {
					this.sendEvent("stopOnStep");
					return true;
				}
			}

			if (stepEvent == "stopOnStep") {
				if (ln == 0) {
					this.sendEvent("stopOnStep");
					return true;
				}
				if (reverse) {
					this.sendEvent("stopOnStep");
					return true;
				}
				if (this.lastManualIndex != -1) {
					let lmStacktrace = this.stacktraces[this.lastManualIndex];
					let curStacktrace = this.stacktraces[ln];

					if (curStacktrace.length < lmStacktrace.length) {
						this.sendEvent("stopOnStep");
						return true;
					}

					if (curStacktrace.length == lmStacktrace.length) {
						if (curStacktrace[0].fileName == lmStacktrace[0].fileName) {
							this.sendEvent("stopOnStep");
							return true;
						}
					}
				}
			}
		}

		// nothing interesting found -> continue
		return false;
	}

	private sendEvent(event: string, ...args: any[]) {
		this.emit(event, ...args);
	}
}
