/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/*
 * TODO: Handle Theme Change.
 */

import { injectable, inject, postConstruct } from "inversify";
import { Emitter, Event } from "@theia/core/lib/common";
import { Process } from "./process";
import { ProcessManager } from "./process-manager";

import * as XTerm from "xterm";

import Terminal = XTerm.Terminal;

@injectable()
export class ProcessWidgetState {
    protected selection: number | undefined;
    protected terminals: Map<number, Terminal>;

    protected readonly changeEmitter: Emitter<void>;

    @inject(ProcessManager)
    protected readonly processManager: ProcessManager;

    public constructor() {
        this.terminals = new Map();
        this.selection = undefined;
        this.changeEmitter = new Emitter<void>();
    }

    @postConstruct()
    protected init(): void {
        this.processManager.onAdd(this.onProcessAdded.bind(this));
        this.processManager.onDelete(this.onProcessDeleted.bind(this));
    }

    protected onProcessAdded(id: number): void {
        const terminal = this.createTerminal();
        const process = this.processManager.getProcess(id);

        this.terminals.set(id, terminal);
        this.bindProcessEvents(process);
        this.bindProcessToTerminal(process, terminal);

        this.select(id);
    }

    protected onProcessDeleted(id: number): void {
        const terminal = this.terminals.get(id);

        if (terminal && this.terminals.delete(id)) terminal.dispose();

        const selection = this.getNextSelection();

        this.select(selection);
    }

    protected bindProcessEvents(process: Process | undefined): void {
        if (process) {
            const event = () => {
                if (process.id === this.selection) this.emitOnChange();
            };

            process.onExit(event);
            process.onError(event);
        }
    }

    protected bindProcessToTerminal(process: Process | undefined, terminal: Terminal | undefined): void {
        if (process && terminal) {
            process.onOutput(data => terminal.write(data));
            process.onErrorOutput(data => terminal.write(data));
        }
    }

    public select(id: number | undefined): void {
        if (this.selection !== id) {
            this.selection = id;

            this.emitOnChange();
        }
    }

    public getSelection(): number | undefined {
        return this.selection;
    }

    protected getNextSelection(): number | undefined {
        if (this.terminals.size > 0) {
            const ids = this.terminals.keys();
            const id = ids.next();

            return id.value;
        }
    }

    public getSelectionAsProcess(): Process | undefined {
        if (this.selection !== undefined) return this.processManager.getProcess(this.selection);
    }

    public getSelectionAsTerminal(): Terminal | undefined {
        if (this.selection !== undefined) return this.terminals.get(this.selection);
    }

    public get onChange(): Event<void> {
        return this.changeEmitter.event;
    }

    protected emitOnChange(): void {
        this.changeEmitter.fire(undefined);
    }

    protected createTerminal(): Terminal {
        const cssProps = this.getCSSPropertiesFromPage();

        return new Terminal({
            cursorBlink: false,
            disableStdin: true,
            fontFamily: cssProps.fontFamily,
            fontSize: cssProps.fontSize,
            theme: {
                foreground: cssProps.foreground,
                background: cssProps.background,
                cursor: cssProps.foreground,
                selection: cssProps.selection
            }
        });
    }

    // tslint:disable-next-line:no-any
    protected getCSSPropertiesFromPage(): any {
        const cssProps = getComputedStyle(document.documentElement);
        const fontSizeStr = cssProps.getPropertyValue("--theia-code-font-size").trim();
        const fontSizeMatch = fontSizeStr.trim().match(/^(\d+)px$/);
        const fontSize =  fontSizeMatch ? parseInt(fontSizeMatch[1], 10) : 12;

        return {
            fontFamily: cssProps.getPropertyValue("--theia-code-font-family").trim(),
            fontSize: fontSize,
            foreground: cssProps.getPropertyValue("--theia-ui-font-color1").trim(),
            background: cssProps.getPropertyValue("--theia-layout-color0").trim(),
            selection: cssProps.getPropertyValue("--theia-transparent-accent-color2").trim()
        };
    }
}
