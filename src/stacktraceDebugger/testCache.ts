import { emaStacktraces, parseStacktraces } from "./stacktraceParser";
import { readFileSync, statSync, readdirSync, writeFileSync } from "fs";
import { isAbsolute } from "path";
import { EMATestRunner } from "./testRunner";

export class TestCache {
	private lastModifiedMap: Object;
	private testRunner: EMATestRunner;

	constructor(testRunner:EMATestRunner) {
		this.testRunner = testRunner;
	}

	public getStacktraces(componentName:string):emaStacktraces | null{
		const targetDir = this.testRunner.getTargetDir(componentName);
		const stacktraceFile = targetDir + "/build/stacktrace.log";
		const lastDebugPath = targetDir + "/lastDebug.json";

		this.testRunner.getLogger().log("Trying to load cached stacktraces...");
		this.loadFileTimestamps(lastDebugPath, componentName);
		let relevantFiles = this.getRelevantFiles(stacktraceFile, lastDebugPath, componentName);

		if (this.hasChangedAndUpdateAll(relevantFiles)) {
			let success = this.doRunTests(componentName, stacktraceFile, lastDebugPath);
			if(!success){
				return null;
			}
		}else{
			this.testRunner.getLogger().log("Files have not changed => Using cache");
		}

		return parseStacktraces(stacktraceFile, this.testRunner.getModelBasePath());

	}

	private doRunTests(componentName: string, stacktraceFile: string, lastDebugPath: string): boolean{
		this.testRunner.getLogger().log("Files have changed => Rerunning tests");
		const testWasRun = this.testRunner.executeTests(componentName);
		if(testWasRun){
			this.hasChangedAndUpdate(stacktraceFile);
			console.log("Writing lastDebug");
			console.log(componentName);
			console.log(this.lastModifiedMap);
			writeFileSync(lastDebugPath, JSON.stringify({"program":componentName, "fileModifiedStamps": this.lastModifiedMap}), {"encoding":"utf-8"});
			return true;
		}else{
			this.testRunner.getLogger().log("Error getting stacktraces. Aborting!");
			return false;
		}
	}

	private getRelevantFiles(stacktraceFile: string, lastDebugPath: string, componentName: string) {
		let relevantFiles = this.getAllFileNames(this.testRunner.getModelBasePath()).filter(fn => fn.endsWith(".emam") || fn.endsWith(".stream"));
		relevantFiles.push(this.testRunner.getGeneratorJarPath());
		relevantFiles.push(stacktraceFile);
		return relevantFiles;
	}

	private loadFileTimestamps(lastDebugPath: string, componentName: string) {
		try {
			const lastDebugContent = readFileSync(lastDebugPath, { "encoding": "utf-8", "flag": "r" });
			if (lastDebugContent) {
				let lastDebugJson = JSON.parse(lastDebugContent);
				if (lastDebugJson) {
					if (lastDebugJson.hasOwnProperty("program") && lastDebugJson.hasOwnProperty("fileModifiedStamps")) {
						if (componentName === lastDebugJson["program"]) {
							this.lastModifiedMap = lastDebugJson["fileModifiedStamps"];
						}
					}
				}
			}
		}
		catch (e) {
			this.testRunner.getLogger().log("Can not load cache =>  Recompiling!");
		}

		if(this.lastModifiedMap == null){
			this.lastModifiedMap = new Object();
		}
	}

	public hasChangedAndUpdateAll(paths: string[]): boolean {
		let res = false;
		for (let p of paths) {
			const newLocal = this.hasChangedAndUpdate(p);
			res = newLocal || res;
		}
		return res;
	}

	public hasChangedAndUpdate(path: string): boolean {
		let prefix = "";
		if (!isAbsolute(path)) {
			prefix = this.testRunner.getModelBasePath() + "/";
		}
		let fullPath = prefix + path;
		let res = true;
		try {
			let lastTime = statSync(fullPath).mtimeMs;
			if (this.lastModifiedMap.hasOwnProperty(fullPath)) {
				const newLocal = Math.abs(this.lastModifiedMap[fullPath] - lastTime) < Number.EPSILON;
				res = !newLocal;
				if(res){
					this.testRunner.getLogger().log("File " + fullPath + " changed! Timestamp was " + this.lastModifiedMap[fullPath] + " and is " + lastTime + "now");
				}
			} else {
				res = true;
			}
			if (res) {
				this.lastModifiedMap[fullPath] = lastTime;
			}
		} catch (e) {
			console.log("Error while checking file " + path + ":" + e)
		}
		return res;
	}

	private getAllFileNames(dir:string): string[] {
		let tmpDir = dir.endsWith("/") ? dir : dir + "/";
		let files = readdirSync(dir);
		let res: string[] = [];
		files.forEach(file => {
			const nextFile = tmpDir + file;
			if (statSync(nextFile).isDirectory()) {
				this.getAllFileNames(nextFile + '/').forEach(f => res.push(f));
			}
			else {
				res.push(tmpDir + file);
			}
		});
		return res;
	}

}