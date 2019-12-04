# coding=utf-8
__author__ = 'Andrey Lazarev'
from flask import Blueprint, render_template, request
from azimuth.webcommon import getdata
from azimuth_cm import aziniparser
import subprocess
import json

node = Blueprint('trafflight', __name__)

menu = [{"url": "/trafflight", "caption": u"Детектор светофоров"}]

ininame = '/etc/odyssey/Traff.ini'

chdefaults = {
    "x1": "0",
    "x2": "0",
    "y1": "0",    
    "y2": "0",
    "width": "0",
    "height": "0",
    "channeltype": "0",
    "channeltcl": "0"
}

def getini():
    return aziniparser.Ini(open(ininame, 'r'))

def getchannel():
    return int(getini().getvalue('main', 'channelinputid') or 0)

def setchannel(count):
    ini = getini()
    ini.setvalue('main', 'channelinputid', count)
    ini.write(open(ininame,'w'))

@node.route('')
def traff_index():
    data = getdata()
    data['channelinputid'] = getchannel()
    return render_template('trafflight.html', **data)

@node.route('/api/channel/<channel>', methods=['GET', 'POST'])
def channelsettigns(channel):
    if request.method=='POST':
        ini = getini()
        data = json.loads(request.data)
        for param in data:
            if param == 'channelinput':
                ini.setvalue('main', param, data[param])
            else:
                ini.setvalue(chstr(channel), param, data[param])
        ini.write(open(ininame, 'w'))
        subprocess.Popen(['sudo', 'service', 'trafconn', 'restart'])

    ini = getini()
    result_section = ini.getsectionwithdefaults(chstr(channel), defaults=chdefaults)
    return json.dumps(result_section, indent=True)

def chstr(channel):
    return 'c{0}'.format(channel)


def preparevalue(v):
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v

