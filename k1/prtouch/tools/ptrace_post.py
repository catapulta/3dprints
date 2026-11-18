import sys, urllib.request, urllib.parse
url = 'http://127.0.0.1:7125/printer/gcode/script?' + urllib.parse.urlencode({'script': sys.argv[1]})
print(urllib.request.urlopen(urllib.request.Request(url, method='POST'), timeout=3600).read().decode())
