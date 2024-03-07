import requests
import json

url = 'http://192.168.34.89/openDoor'
myobj = {
        "action": "openDoor",
        "parameters": {"robotId": "34"}
        }

x = requests.post(url, json = myobj)
x_dict = json.loads(x.text)
status = x_dict["status"]
door_num = x_dict["data"]["message"]
print(status, door_num)

