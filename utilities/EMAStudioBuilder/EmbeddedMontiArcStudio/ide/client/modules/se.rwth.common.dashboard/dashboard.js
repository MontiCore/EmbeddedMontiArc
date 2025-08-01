/* (c) https://github.com/MontiCore/monticore */
var Dashboard = (function() {
    var CONSTANTS = {
        KEY: "dashboard"
    };

    var properties = {
        projects: [],
        $panel: null,
        $list: null,
        $plus: null
    };

    var methods = {
        init: function() {
            methods.load();
            methods.addEventListeners();

            delete methods.init;
            return methods;
        },

        load: function() {
            /*var data = localStorage.getItem(CONSTANTS.KEY);

            data = data || "[]";
            properties.projects = JSON.parse(data);*/
            properties.projects = [{
                username: "EmbeddedMontiArcStudio",
                reponame: "AutoPilot",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/AutoPilot/v&url=/m/autopilot.zip&openFile=/de/rwth/armin/modeling/autopilot/Autopilot.emam"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "Clustering",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/Clustering/v&url=/m/clustering.zip&openFile=/detection/SpectralClusterer.emam"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "Pump",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/Pump/v&url=/m/pump.zip&openFile=/model/pumpStationExample/PumpStation.ema"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "PacMan",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/PacMan/v&url=/m/pacman.zip&openFile=/de/rwth/pacman/PacManWrapper.emam"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "SuperMario",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/SuperMario/v&url=/m/supermario.zip&openFile=/de/rwth/supermario/SuperMarioWrapper.emam"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "NFPVerification",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/NFPVerification/v&url=/m/nfpverification.zip&openFile=/example/model/Sensors.ema"
            }, {
                username: "EmbeddedMontiArcStudio",
                reponame: "OCLVerification",
                branchname: "",
                href: "/api/load.html?mountPoint=EmbeddedMontiArcStudio/OCLVerification/v&url=/m/oclverification.zip&openFile=/cd/EmbeddedMontiArc.cd"
            }];
        },

        save: function() {
            var data = JSON.stringify(properties.projects);

            localStorage.setItem(CONSTANTS.KEY, data);
        },

        addEventListeners: function() {
            Checker.on("checked", events.onChecked);
        },

        addEventHandlers: function() {
            properties.$list.on("click", ".tick", events.onTickClick);
            properties.$list.on("click", ".trash", events.onTrashClick);
            properties.$plus.on("click", events.onPlusClick);
        },

        getProject: function(index) {
            return properties.projects[index];
        },

        hasProject: function(username, reponame, branchname) {
            var projects = properties.projects;
            var length = projects.length;

            for(var i = 0; i < length; i++) {
                var project = projects[i];

                if(project.username === username
                    && project.reponame === reponame
                    && project.branchname === branchname) {
                    return true;
                }
            }

            return false;
        },

        addProject: function(username, reponame, branchname) {
            var project = {};
            var mountPoint = username + '/' + reponame + '/' + branchname;

            project.username = username;
            project.reponame = reponame;
            project.branchname = branchname;

            Trashbin.remove(mountPoint);
            properties.projects.push(project);
            methods.save();
            methods.addListItem(username, reponame, branchname);
            Menu.hide();
        },

        removeProject: function(index, callback) {
            var project = methods.getProject(index);
            var mountPoint = project.username + '/' + project.reponame + '/' + project.branchname;

            Trashbin.add(mountPoint);
            properties.projects.splice(index, 1);
            methods.save();
            methods.removeListItem(index + 1);
            callback(null);
        },

        addListItem: function(username, reponame, branchname, href) {
            var $username = $("<span/>", { "class": "text", "text": username });
            var $reponame = $("<span/>", { "class": "text", "text": reponame });
            //var $branchname = $("<span/>", { "class": "text", "text": branchname });
            var $tick = $("<span/>", { "class": "tick", "href": href });
            //var $trash = $("<span/>", { "class": "trash" });
            var $item = $("<li/>").append($username, $reponame, /*$branchname,*/ $tick/*, $trash*/);

            properties.$list.append($item);
        },

        removeListItem: function(index) {
            properties.$list.children().eq(index).remove();
        },

        addListItems: function() {
            var projects = properties.projects;
            var length = projects.length;

            for(var i = 0; i < length; i++) {
                var project = projects[i];

                methods.addListItem(project.username, project.reponame, project.branchname, project.href);
            }
        },

        show: function() {
            properties.$panel.show();
        },

        hide: function() {
            properties.$panel.hide();
        }
    };

    var events = {
        onChecked: function(event) {
            properties.$panel = $("#dashboard-panel");
            properties.$list = $("#dashboard-list");
            properties.$plus = $("#dashboard-plus");

            methods.addEventHandlers();

            Loader.message("Loading Dashboard...");
            methods.addListItems();
            Loader.hide();
        },

        onTickClick: function(event) {
            /*var index = $(this).index(".tick");
            var project = methods.getProject(index);

            localStorage.setItem("username", project.username);
            localStorage.setItem("reponame", project.reponame);
            localStorage.setItem("branchname", project.branchname);

            window.location.href = "ide.html";*/

            var url = $(this).attr("href");

            window.location.href = url;
        },

        onTrashClick: function(event) {
            var index = $(this).index(".trash");

            function onRemove(error) {
                if(error) Dialog.show("An error occurred while deleting the project.", "error", ["OK"], [], error);
                Loader.hide();
            }

            function onAccept() {
                Loader.show("Deleting Project...");
                methods.removeProject(index, onRemove);
            }

            Dialog.show("Are you sure you want to delete this project?", "warning", ["Yes", "No"], [onAccept]);
        },

        onPlusClick: function(event) {
            Menu.show();
        }
    };

    return methods.init();
})();
