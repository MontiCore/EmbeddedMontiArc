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

function initHeader() {
    $(tableReference + ' thead tr').clone(true).appendTo(tableReference + ' thead');
    $(tableReference + ' thead tr:eq(1)').addClass('group');
    $(tableReference + ' thead tr:eq(1) th').each(function (i) {
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

function initFloatingHeader() {
    var init = false;
    var lastIndex = -1;
    $(document).scroll(function () {
        var index = 0;
        var lastFoundIndex = -1;
        var lastFound = "";
        $(tableReference + ' tbody .group td').each(function () {

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
}

function initLogMechanic(controlClass, formatFunction) {
    // Add event listener for opening and closing details
    $(tableReference + ' tbody').on('click', 'td.' + controlClass, function () {
        var tr = $(this).closest('tr');
        var row = table.row(tr);
        if (row.child.isShown()) {
            // This row is already open - close it
            row.child.hide();
            tr.removeClass('logShown');
        } else {
            // Open this row
            row.child(formatFunction(row.data())).show();
            tr.addClass('logShown');
        }
    });
}

function formatLog1(d) {
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

function initGrowMechanic(growClass, shortLabel, fullLabel) {
    $('.' + growClass).mouseover(function () {
        $(this).find('.' + shortLabel).hide();
        $(this).find('.' + fullLabel).show();
    });
    $('.grow').mouseout(function () {
        $(this).find('.' + shortLabel).show();
        $(this).find('.' + fullLabel).hide();
    });
}