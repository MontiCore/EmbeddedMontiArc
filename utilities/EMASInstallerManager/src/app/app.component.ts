/* (c) https://github.com/MontiCore/monticore */
import { Component, OnInit } from "@angular/core";
import { remote } from "electron";
import { LoadService } from "@services/common/load.service";

@Component({
    selector: "app-root",
    templateUrl: "./app.component.html",
    styleUrls: ["./app.component.scss"]
})
export class AppComponent implements OnInit {
    protected loading: boolean;

    public constructor(protected readonly service: LoadService) {
        this.loading = true;
    }

    public async ngOnInit(): Promise<void> {
        await this.service.load();

        this.loading = false;
    }

    public isLoading(): boolean {
        return this.loading;
    }

    public onCloseClick(): void {
        remote.BrowserWindow.getFocusedWindow().close();
    }

    public onMinimizeClick(): void {
        remote.BrowserWindow.getFocusedWindow().minimize();
    }
}
