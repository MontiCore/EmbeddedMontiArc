/*class Project extends ListItem {
    protected mountPoint: string;

    protected constructor(mountPoint: string) {
        super();
        this.setMountPoint(mountPoint);
    }

    public static async create(mountPoint: string): Promise<Project> {

    }

    public setMountPoint(mountPoint: string): void {
        this.mountPoint = mountPoint;
    }

    public getMountPoint(): string {
        return this.mountPoint;
    }

    public getUsername(): string {
        const parts = this.mountPoint.split('/');

        return parts[0] || '';
    }

    public getReponame(): string {
        const parts = this.mountPoint.split('/');

        return parts[1] || '';
    }

    public getBranchname(): string {
        const parts = this.mountPoint.split('/');

        return parts[2] || '';
    }
}*/

import {UIModule} from "../se.rwth.common.module/module";
import {Loader} from "../se.rwth.common.loader/loader";
import {Dialog} from "../se.rwth.common.dialog/dialog";

export class Dashboard extends UIModule {
    protected loader: Loader;
    protected dialog: Dialog;

    protected $panel: JQuery;
    protected $list: JQuery;
    protected $itemPlus: JQuery;

    protected mountPoints: string[];

    protected constructor() {
        super("common.dashboard");
    }

    protected setDependencies(): void {
        this.loader = Loader.create();
        this.dialog = Dialog.create();
    }

    protected setHTMLElements(): void {
        this.$panel = jQuery("#dashboard-panel");
        this.$list = jQuery("#dashboard-list");
        this.$itemPlus = jQuery("#dashboard-plus");
    }

    protected addEventListeners(): void {
    }

    protected static instance: Dashboard;

    public static async create(): Promise<Dashboard> {
        return Dashboard.instance = Dashboard.instance ? Dashboard.instance : new Dashboard();
    }


    public show(): void {
        this.$panel.show();
    }

    public hide(): void {
        this.$panel.hide();
    }

    public getMountPoint(index: number): any {
        return this.mountPoints[index];
    }

    public hasMountPoint(smountPoint: string): boolean {
        for(let mountPoint of this.mountPoints) {
            if(mountPoint === smountPoint) return true;
        }

        return false;
    }

    public addMountPoint(mountPoint: string): void {
        const mountPoints = this.mountPoints;
        const index = mountPoints.length;

        mountPoints.push(mountPoint);
        this.emit("add", mountPoint, index);
    }

    public removeMountPoint(index: number): void {
        const mountPoints = this.mountPoints;
        const mountPoint = mountPoints[index];

        mountPoints.splice(index, 1);
        this.emit("remove", mountPoint, index);
    }


    private static readonly KEY: string = "dashboard";

    public load(): void {
        const data = window.localStorage.getItem(Dashboard.KEY);

        this.mountPoints = JSON.parse(data);
    }

    public save(): void {
        const data = JSON.stringify(this.mountPoints);

        window.localStorage.setItem(Dashboard.KEY, data);
    }

    private addListItems(): void {
        for(let mountPoint of this.mountPoints) {
            this.addListItem(mountPoint);
        }
    }

    private addListItem(mountPoint: string): void {
        const parts = mountPoint.split('/');
        const $username = jQuery("<span/>", { "class": "text", "text": parts[0] || '' });
        const $reponame = jQuery("<span/>", { "class": "text", "text": parts[1] || '' });
        const $branchname = jQuery("<span/>", { "class": "text", "text": parts[2] || '' });
        const $tick = $("<span/>", { "class": "tick" });
        const $trash = $("<span/>", { "class": "trash" });
        const $item = $("<li/>").append($username, $reponame, $branchname, $tick, $trash);

        this.$list.append($item);
    }

    private removeListItem(index: number): void {
        this.$list.children().eq(index).remove();
    }


    private onChecked(): void {
        this.loader.message("Loading Dashboard...");
        this.addListItems();
        this.loader.hide();
    }

    private onTickClick(event): void {
        const index = $(this).index(".tick");
        const mountPoint = this.getMountPoint(index);

        window.location.href = "ide.html?mountPoint=" + mountPoint;
    }

    private onTrashbinClick(): void {
        const index =
    }

    private onPlusClick(): void {
        this.emit("");
    }
}