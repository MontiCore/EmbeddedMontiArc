/* (c) https://github.com/MontiCore/monticore */
function loadJSON(file, callback) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            callback(xobj.responseText);
        }
    };
    xobj.send(null);
}

function createTable(data) {
    data = JSON.parse(data);
    table = $('#my-table').DataTable({
        "fnInitComplete": function() { init(); },
        "fixedHeader": true,
        "data": data,
        "language": {
            "loadingRecords": "Please wait - loading..."
        },
        "processing": true,
        "orderCellsTop": true,
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {
            "pre": [0, 'asc']
        },
        "select": true,
        "rowGroup": {
            dataSrc: "Root",
            startRender: function (rows, group) {
                return '<div class="nowrap" style="width:0px; overflow:visible;">' + group + ' (' + rows.count() + ')</div>';
            },
            endRender: null //function( rows, group ) {return group;}
        },
        "bAutoWidth": false,
        "aoColumns": [
            {"data": "Root", visible: false},
            {"data": null, visible: false},
            {"data": "Name", sort: "string", type: "alt-string"}
        ],
        "order": [[1, 'asc']]
    });
};

function init() {
    table = $(tableReference).DataTable();
    initFloatingHeader(0);
}


