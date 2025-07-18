/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { RawProcess, RawProcessFactory } from "@theia/process/lib/node";
import { inject, injectable } from "inversify";
import { Server as HTTPServer } from "http";
import { Server as HTTPSServer } from "https";
import { GUIProcessCLIContribution } from "./gui-process-cli-contribution";

@injectable()
export class GUIProcessContribution implements BackendApplicationContribution {
    @inject(RawProcessFactory) protected readonly factory: RawProcessFactory;
    @inject(GUIProcessCLIContribution) protected readonly cli: GUIProcessCLIContribution;

    protected xserver: RawProcess;

    public onStart(server: HTTPServer | HTTPSServer): void {
        this.launchProcess();
    }

    public onStop(): void {
        if (this.xserver) this.xserver.kill();
    }

    protected launchProcess(): void {
        const resolution = this.cli.getResolution();

        this.xserver = this.factory({
            command: "xpra",
            args: [
                "start", "--windows=yes", "--pulseaudio=no", "--daemon=no", "--bind-tcp=0.0.0.0:10000",
                `--xvfb="/usr/bin/Xvfb +extension Composite -screen 0 ${resolution}x24+32 -nolisten tcp -noreset"`,
                "--html=on", "--notifications=no", "--bell=no", "--webcam=no", "--mdns=no", "--dbus-launch=no",
                "--resize-display=yes", "--mousewheel=on", "--sharing=no", ":10"
            ],
            options: { shell: true }
        });

        this.xserver.onExit(() => this.launchProcess());
    }
}
