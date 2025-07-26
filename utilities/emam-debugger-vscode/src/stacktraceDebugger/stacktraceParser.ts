/* (c) https://github.com/MontiCore/monticore */
import { readFileSync } from "fs";

export function parseStacktraces(fileName: string, modelBaseDir: string): emaStacktraces {
	var content: string = readFileSync(fileName, {"encoding":"utf-8", "flag":"r"}).toString();
	return parseStacktracesFromString(content, modelBaseDir);
}

export function parseStacktracesFromString(content: string, modelBaseDir: string): emaStacktraces {
	const enteringPrefix = "~Entering ";
	const tickPrefix = "#tick ";

	var currentTestFile = "unknown";
	var currentTestInstanceName = "unknown";
	var curTick: number = -1;
	var lastTick: number = -1;

	var res: emaStacktraces = new emaStacktraces();
	var lines: string[] = content
		.split("\n")
		.filter(l => l.length != 0)
		.map(l => l.trim())

	var sourcePosRegEx: RegExp = /^at ([^\(]*)\(([^:]*):(\d*)\)$/m;
	var variableRegEx: RegExp = new RegExp('^([^:]*) : ([^$]*)$');

	var curStacktrace: emaStacktrace = new emaStacktrace();
	for (var l of lines) {
		if (l === "Breakpoint reached") {
			curStacktrace = new emaStacktrace();
		} else if (l === "endBreakpoint") {
			if (curStacktrace !== null) {
				if(lastTick != curTick){
					var tmpStack = new emaStacktrace();
					const tmpSourcePos = new emaSourcePosition();
					tmpSourcePos.fileName = currentTestFile;
					tmpSourcePos.instanceName = currentTestInstanceName;
					tmpSourcePos.line = 1;
					if(curTick >= 0){
						try{
							var streamContentLines:string[] = readFileSync(modelBaseDir + "/" + currentTestFile, {"encoding":"utf-8", "flag":"r"}).toString().split("\n");
							var tickLine: string | null = null;
							var i = 0;
							for(var s of streamContentLines){
								if(s.includes(" tick ")){
									tickLine = s;
									break;
								}
								i++;
							}

							if(tickLine){
								tmpSourcePos.line = i + 1;
								var regex = /tick/gi, result: RegExpExecArray | null, indices: Array<number> = [];
								while ((result = regex.exec(tickLine)) ) {
									if(result != null){
										indices.push(result.index);
									}
								}
								if(curTick == 0){
									var colonIndex = tickLine.search(/:/g);
									tmpSourcePos.col = colonIndex == -1 ? 0 : colonIndex + 1;
								}else{
									tmpSourcePos.col = indices[curTick - 1] + 4;
								}
								// filter out whitespace to the left
								var j = tmpSourcePos.col;
								while(j < tickLine.length && /\s/.test(tickLine[j])){
									j++;
								}
								tmpSourcePos.col = j;

								if(curTick < indices.length){
									tmpSourcePos.endCol = indices[curTick];
								}else{
									tmpSourcePos.endCol = tickLine.length - 1;
								}
								// filter out whitespace to the right
								var k = tmpSourcePos.endCol - 1;
								while(k >= 0 && /\s/.test(tickLine[k])){
									k--;
								}
								tmpSourcePos.endCol = k + 1;
							}
						}catch(e){

						}
					}

					tmpStack.push(tmpSourcePos);
					res.push(tmpStack);
				}
				lastTick = curTick;


				const testFileSourcePos = new emaSourcePosition();
				testFileSourcePos.fileName = currentTestFile;
				testFileSourcePos.instanceName = currentTestInstanceName;
				testFileSourcePos.line = 0;
				curStacktrace.push(testFileSourcePos);
				curStacktrace.tick = curTick;
				res.push(curStacktrace);
				curStacktrace = new emaStacktrace();
			}
		} else if (l.startsWith(enteringPrefix)) {
			if(currentTestInstanceName !== "unknown"){
				var tmpStack = new emaStacktrace();
				const tmpSourcePos = new emaSourcePosition();
				tmpSourcePos.fileName = currentTestFile;
				tmpSourcePos.instanceName = currentTestInstanceName;
				tmpSourcePos.line = 1000;
				tmpStack.push(tmpSourcePos);
				res.push(tmpStack);
			}

			currentTestInstanceName = l.replace("~Entering ", "").trim();
			currentTestFile = currentTestInstanceName.replace(/\./g, "/") + ".stream";
		} else if (l.startsWith(tickPrefix)) {
			const tickStr = l.replace(tickPrefix, "");
			curTick = parseInt(tickStr);
			curTick = isNaN(curTick) ? -1 : curTick;
		} else {
			var sourcePosMatch = sourcePosRegEx.exec(l);
			if (sourcePosMatch) {
				var tmp: emaSourcePosition = new emaSourcePosition();
				tmp.instanceName = sourcePosMatch[1];
				tmp.fileName = sourcePosMatch[2];
				const tmpLine = parseInt(sourcePosMatch.length > 3 ? sourcePosMatch[3] : "");
				tmp.line = isNaN(tmpLine) ? 1 : tmpLine;
				curStacktrace.push(tmp);
			}

			var variableMatch = variableRegEx.exec(l);
			if (variableMatch) {
				var tmpVar: emaVariable = new emaVariable();
				tmpVar.name = variableMatch[1];
				tmpVar.value = variableMatch[2];
				curStacktrace.vars.push(tmpVar);
			}
		}
	}
	if(currentTestInstanceName !== "unknown"){
		var tmpStack = new emaStacktrace();
		const tmpSourcePos = new emaSourcePosition();
		tmpSourcePos.fileName = currentTestFile;
		tmpSourcePos.instanceName = currentTestInstanceName;
		tmpSourcePos.line = 1000;
		tmpStack.push(tmpSourcePos);
		res.push(tmpStack);
	}

	return res;
}

export class emaStreamTestResult extends Array<emaStacktrace>{
	name: string;
}

export class emaStacktrace extends Array<emaSourcePosition>{
	vars: emaVariable[] = [];
	tick: number = -1;
}

export class emaStacktraces extends Array<emaStacktrace>{
	public toTestResults(): emaStreamTestResult[] {
		var res: emaStreamTestResult[] = [];

		var tmp: emaStreamTestResult = new emaStreamTestResult();
		var lastTestcaseName = "";
		for (var st of this) {
			var curPos = st[st.length - 1];
			if (curPos.instanceName !== lastTestcaseName) {
				if (tmp.length > 0) {
					res.push(tmp);
				}
				tmp = new emaStreamTestResult();
			}

			lastTestcaseName = curPos.instanceName;
			tmp.push(st)
			tmp.name = lastTestcaseName;
		}
		return res;
	}
}

export class emaSourcePosition {
	fileName: string;
	instanceName: string;
	line: number;
	col?: number;
	endCol?: number;
}

export class emaVariable {
	name: string;
	value: string;
}
