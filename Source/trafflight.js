fields = [
    'x1',
    'y1',
    'x2',
    'y2',
    'width',
    'height',
    'channeltype',
    'channeltcl',
    'channelinput'
];


channel_data = {};
current_channel = 1;

function getDataSuccess(data){
    channel_data = data;    
    for (d in data)
    {
        if (d == "channeltype")
        {
            channel_type = parseInt(data[d]);
            $("#channeltype").val(channel_type);
        }
        else if (d == "channeltcl")
        {
            channel_tcl = parseInt(data[d]);
            $("#channeltcl").val(channel_tcl);
        }
        else
        {
            $('#' + d).val(data[d]);
        }
    }
}
function getDetectChannelParams(channel){
    current_channel = parseInt(channel.value);
    if (!current_channel)
        current_channel = parseInt(channel);
    $.ajax({
        "method": "GET",
        "url": "/trafflight/api/channel/" + (current_channel),
        "dataType":"json",
        "async": false,
        "success": getDataSuccess
    })
}

function saveSettings() {
    channel_data = getDataFromInputs();
    $.ajax({
        method: "POST",
        url: "/trafflight/api/channel/"+(current_channel),
        contentType: "application/json",
        dataType:"json",
        data: JSON.stringify(channel_data),
        async: false,
        success: getDataSuccess
    })
}
function getDataFromInputs()
{
    var data = {};
    for (f in fields)
    {
        data[fields[f]] = parseInt($('#'+fields[f]).val());
        channel_data[fields[f]] = data[fields[f]];
    }
    return data;
}

function ChannelChange(channel)
{
    current_channel = parseInt(channel.value);
    $('#traffin').attr('src', '/video?channel=' + current_channel);
    $('#SelectVideoImg').attr('src', '/video?channel=' + current_channel);
}
function ChannelLoad(channel)
{
    current_channel = parseInt(channel);
    $('#traffin').attr('src', '/video?channel=' + current_channel);
    $('#SelectVideoImg').attr('src', '/video?channel=' + current_channel);
    getDetectChannelParams(1);
}
function preview(img, selection) 
{
    if (!selection.width || !selection.height)
        return;
    
    var scaleX = 200 / selection.width;
    var scaleY = 200 / selection.height;

    $('#preview img').css({
        width: Math.round(scaleX * 400),
        height: Math.round(scaleY * 288),
        marginLeft: -Math.round(scaleX * selection.x1),
        marginTop: -Math.round(scaleY * selection.y1)
    });

    $('#x1').val(selection.x1);
    $('#y1').val(selection.y1);
    $('#x2').val(selection.x2);
    $('#y2').val(selection.y2);
    $('#width').val(selection.width);
    $('#height').val(selection.height);
}
function restartService()
{
    $.ajax({
        "method": "GET",
        "url": "/pedestrians/api/restartservice",
        "dataType":"json",
        "async": true,
        "success": function(data){
            console.log('service restarted');
        }
    })
}


