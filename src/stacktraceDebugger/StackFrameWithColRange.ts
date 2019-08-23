/* (c) https://github.com/MontiCore/monticore */
import { Source } from 'vscode-debugadapter';
import { DebugProtocol } from 'vscode-debugprotocol';
export class StackFrameWithColRange implements DebugProtocol.StackFrame {
	id: number;
	source: Source;
	line: number;
	column: number;
	name: string;
	endColumn: number;
	constructor(i: number, nm: string, src?: Source, ln?: number, col?: number, endCol?: number) {
		this.id = i;
		if (src) {
			let tmpSrcPath = src.path.replace("\/\/", "/");
			this.source = src;
			this.source.path = tmpSrcPath;
		}
		if (ln) {
			this.line = ln;
		}
		if (col) {
			this.column = col;
		}
		this.name = nm;
		if (endCol) {
			this.endColumn = endCol;
		}
	}
}
