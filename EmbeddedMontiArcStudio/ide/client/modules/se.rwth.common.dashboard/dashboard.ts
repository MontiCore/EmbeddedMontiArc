/* (c) https://github.com/MontiCore/monticore */
import {UIModule} from "../se.rwth.common.module/module";
import {Loader} from "../se.rwth.common.loader/loader";
import {Dialog} from "../se.rwth.common.dialog/dialog";
import {Checker} from "../se.rwth.common.checker/checker";

interface Project {
    username: string;
    reponame: string;
    branchname: string;
}

export class Dashboard extends UIModule {
    protected loader: Loader;
    protected dialog: Dialog;
    protected checker: Checker;

    protected $panel: JQuery;
    protected $list: JQuery;
    protected $itemPlus: JQuery;

    protected projects: Project[];

    protected constructor() {
        super("common.dashboard");
        this.load();
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
        this.$list.on("click", ".tick", this.onTickClick);
        this.$list.on("click", ".trash", this.onTrashbinClick);
        this.checker.on("checked", this.onChecked);
        this.$itemPlus.on("click", this.onPlusClick);
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

    public getProject(index: number): Project {
        return this.projects[index];
    }

    public hasProject(username: string, reponame: string, branchname: string): boolean {
        for(let project of this.projects) {
            if(project.username === username
            && project.reponame === reponame
            && project.branchname === branchname) {
                return true;
            }
        }

        return false;
    }

    public addProject(username: string, reponame: string, branchname: string): void {
        const projects = this.projects;
        const index = projects.length;
        const project = <any>{};

        project.username = username;
        project.reponame = reponame;
        project.branchname = branchname;

        projects.push(project);
        this.emit("add", project, index);
    }

    public removeProject(index: number): void {
        const projects = this.projects;
        const project = projects[index];

        this.loader.show("Deleting Project...");
        projects.splice(index, 1);
        this.emit("remove", project, index);
        this.loader.hide();
    }


    private static readonly KEY: string = "dashboard";

    public load(): void {
        const data = window.localStorage.getItem(Dashboard.KEY);

        this.projects = JSON.parse(data);

        this.emit("load");
    }

    public save(): void {
        const data = JSON.stringify(this.projects);

        window.localStorage.setItem(Dashboard.KEY, data);
        this.emit("save");
    }

    private addListItems(): void {
        for(let project of this.projects) {
            this.addListItem(project.username, project.reponame, project.branchname);
        }
    }

    private addListItem(username: string, reponame: string, branchname: string): void {
        const $username = jQuery("<span/>", { "class": "text", "text": username });
        const $reponame = jQuery("<span/>", { "class": "text", "text": reponame });
        const $branchname = jQuery("<span/>", { "class": "text", "text": branchname });
        const $tick = $("<span/>", { "class": "tick" });
        const $trash = $("<span/>", { "class": "trash" });
        const $item = $("<li/>").append($username, $reponame, $branchname, $tick, $trash);

        this.$list.append($item);
    }

    private removeListItem(index: number): void {
        this.$list.children().eq(index).remove();
    }


    private onChecked(): void {
        this.loader.show("Loading Dashboard...");
        this.addListItems();
        this.loader.hide();
    }

    private onTickClick(event): void {
        const index = $(this).index(".tick");
        const project = this.getProject(index);
        const mountPoint = project.username + '/' + project.reponame + '/' + project.branchname;

        window.location.href = "ide.html?mountPoint=" + mountPoint;
    }

    private onTrashbinClick(): void {
        const index = $(this).index(".trash");

        const onDialogClick = (buttonIndex: number) => {
            if(buttonIndex === 0) this.removeProject(index);
        };

        this.dialog.once("click", onDialogClick);
        this.dialog.show("Are you sure you want to delete this project?", "warning", ["Yes", "No"]);
    }

    private onPlusClick(): void {
        this.emit("plusClick");
    }
}
