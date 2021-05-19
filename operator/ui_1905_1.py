from communication_operator import *

# import for GIS
from kivy_garden.mapview import MapView

# imports for UI
from kivy.app import App

from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout

from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.scrollview import ScrollView
from kivy.uix.image import Image
from kivy.uix.textinput import TextInput

import json


def callback(msg_type, msg):
    if msg_type == 'text':
        msg = json.load(msg)
        UI.DRONE_LIST.children[DroneButton.IDs.index(msg["id"])].set_data(
            msg["coords"]["x"],
            msg["coords"]["y"],
            msg["coords"]["z"],
            msg["charge"]
        )
    elif msg_type == 'image_path':
        UI.IMAGE.source = msg


class DroneButton(Button):

    IDs = []

    def __init__(self, ID):
        super().__init__()
        self.ID = ID
        self.x_coord = None
        self.y_coord = None
        self.z_coord = None
        self.charge = None

        self.text = f'Drone {ID}'
        self.size_hint_y = None
        self.height = 40

        DroneButton.IDs.insert(0, ID)

    def set_data(self, x, y, z, charge):
        self.x_coord = x
        self.y_coord = y
        self.z_coord = z
        self.charge = charge

    def on_press(self):
        UI.INFO_ID.text = f'{self.ID}'
        UI.INFO_X.text = f'{self.x_coord}' if self.x_coord is not None else ''
        UI.INFO_Y.text = f'{self.y_coord}' if self.y_coord is not None else ''
        UI.INFO_Z.text = f'{self.z_coord}' if self.z_coord is not None else ''
        UI.INFO_CHRG.text = f'{self.charge}' if self.charge is not None else ''


class SendButton(Button):
    def __init__(self):
        super().__init__()
        self.text = 'SEND MESSAGE TO DRONES'
        self.size_hint_y = 0.1

    def on_press(self):
        if UI.DATA_INPUT.text:
            send_message_to_all(UI.DATA_INPUT.text)


class UI(App):

    def build(self):

        base_layout = GridLayout(
            spacing=10,
            orientation='lr-tb',
            rows=1
        )

        left_layout = GridLayout(
            spacing=10,
            orientation='lr-tb',
            cols=1
        )

        right_layout = GridLayout(
            spacing=10,
            orientation='lr-tb',
            cols=1
        )

        left_layout.add_widget(MapView(zoom=16, lat=59.927283, lon=30.338344))

        data_input = TextInput(multiline=False, size_hint_y=0.1)
        UI.DATA_INPUT = data_input

        left_layout.add_widget(data_input)

        left_layout.add_widget(SendButton())

        drone_table = GridLayout(
            spacing=10,
            orientation='lr-tb',
            rows=1,
            size_hint_y=1.5
        )

        drone_list = GridLayout(
            spacing=5,
            cols=1,
            size_hint_y=None
        )
        drone_list.bind(minimum_height=drone_list.setter('height'))
        UI.DRONE_LIST = drone_list

        with open('../configuration.json') as cnfg:
            config = json.load(cnfg)
        for i in range(config['drones count']):
            drone_list.add_widget(DroneButton(ID=i))

        scroll = ScrollView()

        scroll.add_widget(drone_list)

        info_widget = GridLayout(
            spacing=10,
            orientation='lr-tb',
            cols=1
        )
        info_widget.add_widget(Label(text='Drone ID'))
        info_widget.add_widget(Label(text=''))
        info_widget.add_widget(Label(text='X Coord'))
        info_widget.add_widget(Label(text=''))
        info_widget.add_widget(Label(text='Y Coord'))
        info_widget.add_widget(Label(text=''))
        info_widget.add_widget(Label(text='Z Coord'))
        info_widget.add_widget(Label(text=''))
        info_widget.add_widget(Label(text='Charge'))
        info_widget.add_widget(Label(text=''))
        UI.INFO_ID = info_widget.children[8]
        UI.INFO_X = info_widget.children[6]
        UI.INFO_Y = info_widget.children[4]
        UI.INFO_Z = info_widget.children[2]
        UI.INFO_CHRG = info_widget.children[0]

        image_widget = Image()
        UI.IMAGE = image_widget

        drone_table.add_widget(scroll)
        drone_table.add_widget(info_widget)

        # right_layout.add_widget(Button(text='Exit', size_hint_y=0.1, on_press=self.stop))
        right_layout.add_widget(drone_table)
        right_layout.add_widget(image_widget)

        base_layout.add_widget(left_layout)
        base_layout.add_widget(right_layout)

        return base_layout


if __name__ == '__main__':
    create_listener(callback)
    UI().run()
