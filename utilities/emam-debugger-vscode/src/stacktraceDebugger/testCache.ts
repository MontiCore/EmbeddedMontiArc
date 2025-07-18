/* (c) https://github.com/MontiCore/monticore */
import { emaStacktraces, parseStacktraces } from "./stacktraceParser";
import { readFileSync, statSync, readdirSync, writeFileSync, existsSync } from "fs";
import { isAbsolute } from "path";
import { EMATestRunner } from "./testRunner";
import * as log4js from 'log4js';
export class TestCache {
	private lastModifiedMap: Object;
	private testRunner: EMATestRunner;

	constructor(testRunner: EMATestRunner) {
		this.testRunner = testRunner;
	}

	public getStacktraces(componentName: string): emaStacktraces | null {
		log4js.getLogger().trace("getStacktraces");
		const targetDir = this.testRunner.getTargetDir(componentName);
		const stacktraceFile = targetDir + "/build/stacktrace.log";
		const lastDebugPath = targetDir + "/lastDebug.json";

		this.testRunner.getDebugConsoleLogger().log("Trying to load cached stacktraces...");
		this.loadFileTimestamps(lastDebugPath, componentName);
		let relevantFiles = this.getRelevantFiles(stacktraceFile, lastDebugPath, componentName);

		if (this.hasChangedAndUpdateAll(relevantFiles)) {
			let success = this.doRunTests(componentName, stacktraceFile, lastDebugPath);
			if (!success) {
				return null;
			}
		} else {
			this.testRunner.getDebugConsoleLogger().log("Files have not changed => Using cache");
		}
		if(existsSync(stacktraceFile)){
			return parseStacktraces(stacktraceFile, this.testRunner.getModelBasePath());
		}else{
			return null;
		}

	}

	private doRunTests(componentName: string, stacktraceFile: string, lastDebugPath: string): boolean {
		log4js.getLogger().trace("doRunTests");
		this.testRunner.getDebugConsoleLogger().log("Files have changed => Rerunning tests");
		const testWasRun = this.testRunner.executeTests(componentName);
		if (testWasRun) {
			this.hasChangedAndUpdate(stacktraceFile);
			log4js.getLogger().debug("Writing lastDebug");
			log4js.getLogger().trace(componentName);
			log4js.getLogger().trace(this.lastModifiedMap);
			writeFileSync(lastDebugPath, JSON.stringify({ "program": componentName, "fileModifiedStamps": this.lastModifiedMap }), { "encoding": "utf-8" });
			return true;
		} else {
			this.testRunner.getDebugConsoleLogger().log("Error getting stacktraces. Aborting!");
			return false;
		}
	}

	private getRelevantFiles(stacktraceFile: string, lastDebugPath: string, componentName: string) {
		log4js.getLogger().trace("getRelevantFiles");
		let relevantFiles = this.getAllFileNames(this.testRunner.getModelBasePath()).filter(fn => fn.endsWith(".emam") || fn.endsWith(".stream"));
		relevantFiles.push(this.testRunner.getMavenPomPath());
		relevantFiles.push(stacktraceFile);
		return relevantFiles;
	}

	private loadFileTimestamps(lastDebugPath: string, componentName: string) {
		log4js.getLogger().trace("loadFileTimestamps");
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
			this.testRunner.getDebugConsoleLogger().log("Can not load cache =>  Recompiling!");
		}

		if (this.lastModifiedMap == null) {
			this.lastModifiedMap = new Object();
		}
	}

	public hasChangedAndUpdateAll(paths: string[]): boolean {
		log4js.getLogger().trace("hasChangedAndUpdateAll");
		let res = false;
		for (let p of paths) {
			const newLocal = this.hasChangedAndUpdate(p);
			res = newLocal || res;
		}
		return res;
	}

	public hasChangedAndUpdate(path: string): boolean {
		log4js.getLogger().trace("hasChangedAndUpdate");
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
				if (res) {
					log4js.getLogger().debug("File " + fullPath + " changed! Timestamp was " + this.lastModifiedMap[fullPath] + " and is " + lastTime + "now");
				}
			} else {
				res = true;
			}
			if (res) {
				this.lastModifiedMap[fullPath] = lastTime;
			}
		} catch (e) {
			log4js.getLogger().warn("Error while checking file " + path + ":" + e)
		}
		return res;
	}

	private getAllFileNames(dir: string): string[] {
		log4js.getLogger().trace("getAllFileNames");
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
