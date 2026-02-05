import json
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID

ChannelFactoryInitialize(0, "enp3s0")  # Replace with your interface
loco_client = LocoClient()
loco_client.SetTimeout(10.0)
loco_client.Init()

code, data = loco_client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
print(f"Code: {code}, Data: {data}")