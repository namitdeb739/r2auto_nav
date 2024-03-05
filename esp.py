import requests

url = 'http://192.168.34.89/openDoor'
myobj = {
        "action": "openDoor",
        "parameters": {"robotId": "34"}
        }

x = requests.post(url, json = myobj)
print(x.text)
