function loadInfo(infoFile, file, callback, ide) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', infoFile, true);
    var res;
    var ready = 0;
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            res = JSON.parse(xobj.responseText);
            loadJSON(file, callback, res, ide);
        }
    };
    xobj.send(null);
}

function loadJSON(file, callback, info, ide) {
    var xobj = new XMLHttpRequest();
    xobj.overrideMimeType('application/json');
    xobj.open('GET', file, true);
    xobj.onreadystatechange = function () {
        if (xobj.readyState == 4 && xobj.status == '200') {
            // Required use of an anonymous callback as .open will NOT return a value but simply returns undefined in asynchronous mode
            callback(xobj.responseText, info, ide);
        }
    };
    xobj.send(null);
}

function createTable(data, info, ide) {
    data = JSON.parse(data);
    table = $('#my-table').DataTable({
        fixedHeader: true,
        "data": data,
        "language": {
            "loadingRecords": "Please wait - loading..."
        },
        "processing": true,
        "orderCellsTop": true,
        "aaSorting": [5, 'asc'],
        "ordering": true,
        "paging": false,
        "bInfo": false,
        "orderFixed": {
            "pre": [0, 'asc'],
            "pre": [1, 'asc']
        },
        "select": true,
        "rowGroup": {
            dataSrc: "Root",
            startRender: function (rows, group) {
                var count = info[group]['Number'];
                var valid = info[group]['Valid'];
                var invalid = info[group]['Invalid'];
                return group + ' (' + count + ')&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; valid: ' + valid + '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; invalid: ' + invalid;
            },
            endRender: null //function( rows, group ) {return group;}
        },
        "bAutoWidth": false,
        "aoColumns": [
            {"data": "Root", visible: false},
            {"data": "Order", visible: false},
            {
                "className": "expandChildren-control",
                "data": "ChildExpansion",
                "orderable": false,
            },
            {
                "className": 'groupName',
                "data": "Name",
                "sType": "groupName",
                "bSortable": true,
                "sWidth": "50%"
            },
            {
                "data": "OnlineIDE",
                "className": "onlineIDE",
                "orderable": false,
                "visible": ide
            },
            {
                "className": 'details-control',
                "orderable": false,
                "data": null,
                "defaultContent": ''
            },
            {"data": "FileType", sort: "string"},
            {"data": "Valid", sort: "string", type: "alt-string"},
            {"data": "Parse", sort: "string", type: "alt-string"},
            {"data": "Resolve", sort: "string", type: "alt-string"},
            {"data": "ComponentCapitalized", sort: "string", type: "alt-string"},
            {"data": "ComponentInstanceNamesUnique", sort: "string", type: "alt-string"},
            {"data": "ComponentWithTypeParametersHasInstance", sort: "string", type: "alt-string"},
            {"data": "ConnectorEndPointCorrectlyQualified", sort: "string", type: "alt-string"},
            {"data": "DefaultParametersHaveCorrectOrder", sort: "string", type: "alt-string"},
            {"data": "InPortUniqueSender", sort: "string", type: "alt-string"},
            {"data": "PackageLowerCase", sort: "string", type: "alt-string"},
            {"data": "ParameterNamesUnique", sort: "string", type: "alt-string"},
            {"data": "PortTypeOnlyBooleanOrSIUnit", sort: "string", type: "alt-string"},
            {"data": "PortUsage", sort: "string", type: "alt-string"},
            {"data": "ReferencedSubComponentExists", sort: "string", type: "alt-string"},
            {"data": "SimpleConnectorSourceExists", sort: "string", type: "alt-string"},
            {"data": "SourceTargetNumberMatch", sort: "string", type: "alt-string"},
            {"data": "SubComponentsConnected", sort: "string", type: "alt-string"},
            {"data": "TopLevelComponentHasNoInstanceName", sort: "string", type: "alt-string"},
            {"data": "TypeParameterNamesUnique", sort: "string", type: "alt-string"},
            {"data": "AtomicComponent", sort: "string", type: "alt-string"}
        ],
        "fnInitComplete": function(oSettings, json) {
            childControlInit();
        },
        "order": [[1, 'asc']]
    });
};

function formatLog(d) {
    // `d` is the original data object for the row
    return '<tr>' +
        '<td>Path:</td>' +
        '<td>' + d.Path + '</td>' +
        '</tr>' +
        '<tr>' +
        '<td>Log output:</td>' +
        '<td>' + d.LogOutput + '</td>' +
        '</tr>';
}

function incClass(row, childRow) {
    if ($(row).hasClass("childRow1")) {
        $(childRow).addClass("childRow2");
    } else if ($(row).hasClass("childRow2")) {
        $(childRow).addClass("childRow3");
    } else if ($(row).hasClass("childRow3")) {
        $(childRow).addClass("childRow4");
    } else if ($(row).hasClass("childRow4")) {
        $(childRow).addClass("childRow5");
    } else if ($(row).hasClass("childRow5")) {
        $(childRow).addClass("childRow6");
    } else if ($(row).hasClass("childRow6")) {
        $(childRow).addClass("childRow7");
    } else if ($(row).hasClass("childRow7")) {
        $(childRow).addClass("childRow8");
    } else {
        $(childRow).addClass("childRow1");
    }
}

removeChildRows = function (tr, row, data) {
    const table = $('#my-table').DataTable();
    var name = data.Project.replace("/", "") + "_" + data.Name.replace(/\./g, "_").replace(/\(/, "").replace(/\)/,"").split(' ').join('');

    $('.' + name + '_Child').each(function () {
        var childTr = $(this).closest('tr');
        var childRow = table.row(childTr);
        table.row(childRow).remove();
    });

    $(tr).removeClass('childrenShown');
    $(tr).addClass('childrenNotShown');
}

expandRow = function (tr, row, data) {

    if (data.ChildData.length != 0) {


        const table = $('#my-table').DataTable();
        var name = data.Project.replace("/", "") + "_" + data.Name.replace(/\./g, "_").replace(/\(/, "").replace(/\)/,"").split(' ').join('');

        if ($(tr).hasClass('childrenShown')) {
            removeChildRows(tr, row, data);
        } else {
            for (var i = 0; i < data.ChildData.length; i++) {
                const tmpRow = table.row.add(data.ChildData[i]).node();
                incClass(tr[0], tmpRow);
                $(tmpRow).addClass((name + "_Child"));
                $(tmpRow).removeClass('childrenNotShown');
            }
            $(tr).removeClass('childrenNotShown');
            $(tr).addClass('childrenShown');
        }
        table.draw();
    }

};

function childControlInit() {
    var table = $('#my-table').DataTable();

    $('#my-table tbody').on('click', 'td.expandChildren-control', function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);

        expandRow(tr, row, row.data());
    });


    var indexes = table.rows().eq(0).filter(function (rowIdx) {
        var res = (table.cell(rowIdx, 3).data().indexOf('Parsing failed') !== -1 || table.cell(rowIdx, 3).data().indexOf('Resolving failed') !== -1) ? true : false;
        return res;
    });

    table.rows(indexes).eq(0).map(function (rowIdx) {
        table.cell(rowIdx, 2).nodes().to$().click();
    });
}

function adjustHeader() {
    $('#my-table thead tr:eq(1) th').each(function (i) {
        $(this).text('');
    });
}

function adjustFloatingHeader(content) {
    var first = true;
    var second = false;
    $('.fixedHeader-floating thead tr:eq(1) th').each(function (i) {
        if (first) {
            first = false;
            second = true;
        } else if (second) {
            second = false;
            $(this).html(content);
        } else {
            $(this).text('');
        }
    });
}

function getURLParameter(sParam) {
    var sPageURL = window.location.search.substring(1);
    var sURLVariables = sPageURL.split('&');
    for (var i = 0; i < sURLVariables.length; i++) {
        var sParameterName = sURLVariables[i].split('=');
        if (sParameterName[0] == sParam) {
            return decodeURIComponent(sParameterName[1]);
        }
    }
}

$(document).ready(function () {
    jQuery.fn.dataTableExt.oSort["groupName-desc"] = function (x, y) {
        return x < y;
    };

    jQuery.fn.dataTableExt.oSort["groupName-asc"] = function (x, y) {
        return x > y;
    }

    $('#my-table thead tr').clone(true).appendTo('#my-table thead');
    $('#my-table thead tr:eq(1)').addClass('group');
    adjustHeader();

    var ide = (getURLParameter('ide') != 'false');
    loadInfo("data/info.json", "data/data.json", createTable, ide);

    // Add event listener for opening and closing details
    $('#my-table tbody').on('click', 'td.details-control', function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);

        if (row.child.isShown()) {
            // This row is already open - close it
            row.child.hide();
            tr.removeClass('detailsShown');
        } else {
            // Open this row
            row.child(formatLog(row.data())).show();
            tr.addClass('detailsShown');
        }
    });

    var init = false;
    var lastIndex = -1;
    $(document).scroll(function () {
        var currentGroup = "EmbeddedMontiArc";
        var index = 0;
        var lastFoundIndex = -1;
        var lastFound = "";
        $('#my-table tbody .group td').each(function () {

            var docViewTop = $(window).scrollTop();
            var elemTop = $(this).offset().top;
            var elemBottom = elemTop + $(this).height();

            if (elemBottom < docViewTop || elemTop < docViewTop) {
                lastFound = $(this).html();
                lastFoundIndex = index;
            }
            index++;
        });

        if (lastFoundIndex != lastIndex || init == false) {
            lastIndex = lastFoundIndex;
            if ($('.fixedHeader-floating')[0]) {
                adjustFloatingHeader(lastFound);
                init = true;
            }
        }

        if (!$('.fixedHeader-floating')[0]) {
            init = false;
        }
    });

    $('.grow').mouseover(function () {
        $(this).find('.shortLabel').hide();
        $(this).find('.fullLabel').show();
    });
    $('.grow').mouseout(function () {
        $(this).find('.shortLabel').show();
        $(this).find('.fullLabel').hide();
    });
});
