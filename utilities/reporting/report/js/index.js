/* (c) https://github.com/MontiCore/monticore */
function loadInfo(infoFile) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', infoFile, true);
    var res;
    var ready = 0;
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            var projects = JSON.parse(xobj.responseText);
            processInfo(projects);
        }
    };
    xobj.send(null);
}

function processInfo(projects) {
    projects.sort(function (a, b) {
        return a.toLowerCase().localeCompare(b.toLowerCase());
    });
    projects.forEach(function(project) {
        $('#select-project').append(new Option(project, project));
    });
}

$(document).ready( function() {
    loadInfo("report/data/projects/projectsList.json");
    $( "#select-project" ).selectmenu().selectmenu( "menuWidget" ).addClass( "overflow" );
    $( ".widget input[type=submit], .widget a, .widget button" ).button();

    $( "#select-project" ).on("selectmenuchange", function(){
        var newLink = "report/componentQuality.html";
        if ($('#select-project option:selected').val() != -1)
            newLink += "?project=" + $('#select-project option:selected').text();
        $( "#button-cq" ).attr("href", newLink);
    });
} );
