/* (c) https://github.com/MontiCore/monticore */
$(window).resize(function () {
    $('.reportFrame').each(function () {
        resizeIframe(this);
    });
});

function resizeIframe(obj) {
    obj.style.height = $(window).height() + 'px';
    obj.style.width = $(window).width() + 'px';
    obj.style.position = 'absolute';
    obj.style.left = 0 + 'px';
    obj.style.top = 0 + 'px';
    obj.style.margin = 0 + 'px';
    obj.style.padding = 0 + 'px';
}

function tabExpanded() {
    console.log("Expand");
    $('#groupedFrame').css({'visibility': 'hidden'});
    $('#expandedFrame').css({'visibility': 'visible'});
    // $('#groupedFrame').hide();
    // $('#expandedFrame').show();
}

function tabExpandedWithOrder(i) {
    tabExpanded();
    window.frames['expandedFrame'].contentWindow.table
        .order([i, 'asc'])
        .draw();
}

function tabGrouped() {
    console.log("Group");
    $('#groupedFrame').css({'visibility': 'visible'});
    $('#expandedFrame').css({'visibility': 'hidden'});
    // $('#groupedFrame').show();
    // $('#expandedFrame').hide();

}

function expandedFrameInited() {
    console.log("Init complete");
    $('#loadingFrame').css({'display': 'none'});
    $('#groupedFrame').css({'visibility': 'visible'});
}

document.getElementById('groupedFrame').src = "cocosReport.html" + window.location.search;
document.getElementById('expandedFrame').src = "cocosReportExpanded.html" + window.location.search;
