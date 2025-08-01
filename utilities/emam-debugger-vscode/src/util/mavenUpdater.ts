/* (c) https://github.com/MontiCore/monticore */
import { spawn, SpawnOptions } from 'child_process';
import { Logger, getLogger } from 'log4js';
import { ChunkBuffer } from './chunkBuffer';

export class MavenUpdater{
    private pomPath: string;
    private relativeMvnSettingsPath: string | null;
    private logger: Logger;
    private id: string;

    constructor(id :string ,pomPath: string, relativeMvnSettingsPath: string | null = null){
        this.id = id;
        this.pomPath = pomPath;
        this.relativeMvnSettingsPath = relativeMvnSettingsPath;
        this.logger = getLogger("Updater - " + this.id);
    }

	doUpdate(): Promise<number> {

        return new Promise<number>((resolve, reject) => {
            let buffer = new ChunkBuffer(1000);
            this.logger.trace("maven path: " + this.pomPath);
            let spawnOptions: SpawnOptions = {
                cwd: this.pomPath,
                env: process.env,
                stdio: "pipe",
                shell: true
            };
            const args: string[] = [
                "versions:update-properties",
                "-DallowSnapshots=true"
            ];
            if(this.relativeMvnSettingsPath){
                args.push("-s");
                args.push(this.relativeMvnSettingsPath);
            }

            this.logger.debug("Starting update");
            let p = spawn("mvn", args, spawnOptions);

            p.on("error", (err) =>{
                this.logger.error("Error while updating: " + err);
                return reject(err);
            });

            p.on("exit", (code, signal) => {
                if(code === 0){
                    this.logger.info("Update successful");
                }else{
                    this.logger.error("Error while updating: Process exited with code " + code + " from signal " + signal);
                    this.logger.debug(buffer.getText());
                }
                return resolve(code !== null ? code : 1);
            });

            p.stdout.on("data", (data) => buffer.enq(data.toString()));
        });
    }

    isUpdateAvailable(): Promise<MavenUpdater | null>{

        this.logger.trace("maven path: " + this.pomPath);
        let spawnOptions: SpawnOptions = {
            cwd: this.pomPath,
            env: process.env,
            stdio: "pipe",
            shell: true
        };
        const args: string[] = [
            "versions:display-dependency-updates",
            '-U',
            "-DallowSnapshots=true"
        ];
        if (this.relativeMvnSettingsPath) {
            args.push("-s", this.relativeMvnSettingsPath);
        }

        return new Promise((resolve, reject) => {
            let circularBuffer = new ChunkBuffer(1000);
            this.logger.debug("spawning maven process!");
            let updateProcess = spawn("mvn", args, spawnOptions);
            updateProcess.on("error", (err) => {
                this.logger.error("Maven error: " + err);
                reject(null);
            });

            updateProcess.on("exit" ,(code, signal) => {
                this.logger.debug("Maven process exited with code " + code + " and signal " + signal);
                resolve(null);
            });

            updateProcess.stdout.on("data" , (data) => {
                circularBuffer.enq(data.toString());
                if(circularBuffer.getText().includes("The following dependencies in Dependencies have newer versions")){
                    this.logger.debug("Found updates!");
                    circularBuffer.clear();
                    resolve(this);
                }
            });
        });
    }
}
