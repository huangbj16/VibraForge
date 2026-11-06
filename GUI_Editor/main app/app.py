from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import *

import sys
import os
import random
import time
import pickle
import csv
from scipy import signal
import numpy as np

import matplotlib
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
matplotlib.use('QtAgg')
from matplotlib.colors import to_rgba

from python_ble_api import python_ble_api
from signal_segmentation_api import signal_segmentation_api
from utils import *
from timeline_timer import TimelineTimer
from signal_generator import OscillatorDialog, ChirpDialog, NoiseDialog, FMDialog, PWMDialog

class BluetoothDeviceSearchThread(QtCore.QThread):
    devices_found = QtCore.pyqtSignal(list)

    def __init__(self, ble_api):
        super().__init__()
        self.ble_api = ble_api

    def run(self):
        # This method will be executed in a separate thread
        devices = self.ble_api.get_ble_devices()
        self.devices_found.emit(devices)  # Emit the devices found signal

class BluetoothConnectThread(QtCore.QThread):
    connection_result = QtCore.pyqtSignal(bool)

    def __init__(self, ble_api, device_name):
        super().__init__()
        self.ble_api = ble_api
        self.device_name = device_name

    def run(self):
        success = self.ble_api.connect_ble_device(self.device_name)
        self.connection_result.emit(success)  # Emit the result (success or failure)

class BluetoothConnectDialog(QtWidgets.QDialog):
    device_selected_signal = QtCore.pyqtSignal(bool)

    def __init__(self, ble_api, parent=None):
        super(BluetoothConnectDialog, self).__init__(parent)
        self.setWindowTitle("Connect to Bluetooth Device")
        self.ble_api = ble_api

        # Layout for the popup window
        layout = QtWidgets.QVBoxLayout(self)

        # Label for searching status
        self.status_label = QtWidgets.QLabel("Searching For Devices...", self)
        layout.addWidget(self.status_label)

        # Create a horizontal layout for the buttons
        button_layout = QtWidgets.QHBoxLayout()

        # Create a dropdown for available devices
        self.device_dropdown = QtWidgets.QComboBox(self)
        layout.addWidget(self.device_dropdown)

        # Create a Search button
        self.search_button = QtWidgets.QPushButton("Search", self)
        self.search_button.setEnabled(False)  # Disabled while searching
        button_layout.addWidget(self.search_button)

        # Create a Connect button
        self.connect_button = QtWidgets.QPushButton("Connect", self)
        self.connect_button.setEnabled(False)  # Disabled until devices are found
        button_layout.addWidget(self.connect_button)

        layout.addLayout(button_layout)

        # Signal-slot connection for the buttons
        self.connect_button.clicked.connect(self.connect_to_device)
        self.search_button.clicked.connect(self.start_search)

        # Start the initial search for devices in a background thread
        self.start_search()

    def start_search(self):
        """Start searching for devices in the background thread."""
        self.device_dropdown.clear()  # Clear the dropdown list
        self.connect_button.setEnabled(False)  # Disable connect button while searching
        self.status_label.setText("Searching For Devices. Be Right Back...")  # Update status label
        self.search_button.setEnabled(False)  # Disable search button while searching

        # Start the search in a new background thread
        self.search_thread = BluetoothDeviceSearchThread(self.ble_api)
        self.search_thread.devices_found.connect(self.load_devices)
        self.search_thread.start()

    def load_devices(self, devices):
        """Load BLE devices into the dropdown list once the search is complete."""
        if devices:
            self.device_dropdown.addItems(devices)
            self.connect_button.setEnabled(True)  # Enable connect button when devices are found
            self.status_label.setText(f"Found {len(devices)} device(s).")
        else:
            self.status_label.setText("No devices found.")
        self.search_button.setEnabled(True)  # Enable search button after search completes

    def connect_to_device(self):
        """Attempt to connect to the selected device in a background thread."""
        selected_device = self.device_dropdown.currentText()
        if selected_device:
            # Lock the buttons and show "Connecting to device..."
            self.connect_button.setEnabled(False)
            self.search_button.setEnabled(False)
            self.status_label.setText(f"Connecting to {selected_device}...")

            # Start the connection in a background thread
            self.connect_thread = BluetoothConnectThread(self.ble_api, selected_device)
            self.connect_thread.connection_result.connect(self.on_connection_finished)
            self.connect_thread.start()

    def on_connection_finished(self, success):
        """Handle the result of the Bluetooth connection attempt."""
        # Emit the connection result signal
        self.device_selected_signal.emit(success)
    
        # Re-enable the buttons after the attempt
        self.connect_button.setEnabled(True)
        self.search_button.setEnabled(True)

        # Show the connection status
        QtWidgets.QMessageBox.information(self, "Connection Status",
                                        f"Connection {'Succeeded' if success else 'Failed'}!")

        # If the connection succeeds, close the dialog
        if success:
            self.accept()  # Close the dialog only if connection succeeds
        else:
            # Update status to reflect failure but keep the window open
            self.status_label.setText("Connection failed. Please try again.")

    def disconnect_from_device(self):
        """Attempt to disconnect the connected device."""
        # Lock the buttons and show "Disconnecting from device..."
        self.connect_button.setEnabled(False)
        self.search_button.setEnabled(False)
        self.status_label.setText(f"Disconnecting from {self.device_dropdown.currentText()}...")

        # Attempt to disconnect
        success = self.ble_api.disconnect_ble_device()

        # Show the disconnection status
        QtWidgets.QMessageBox.information(self, "Disconnection Status",
                                        f"Disconnection {'Succeeded' if success else 'Failed'}!")

        # Emit signal for future use
        self.device_selected_signal.emit(success)

        # Re-enable the buttons after the attempt
        self.connect_button.setEnabled(True)
        self.search_button.setEnabled(True)

        # If the disconnection succeeds, close the dialog
        if success:
            self.accept()  # Close the dialog only if disconnection succeeds
        else:
            # Update status to reflect failure but keep the window open
            self.status_label.setText("Disconnection failed. Please try again.")

    def show_disconnect_confirmation(self):
        """Show a confirmation dialog before disconnecting."""
        reply = QtWidgets.QMessageBox.question(self, 'Disconnect Bluetooth Device',
                                            "Are you sure you want to disconnect?",
                                            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                                            QtWidgets.QMessageBox.StandardButton.No)
        
        if reply == QtWidgets.QMessageBox.StandardButton.Yes:
            self.disconnect_from_device()  # Call the method to disconnect
        else:
            self.status_label.setText("Disconnection cancelled.")

class HapticCommandManager:
    def __init__(self, ble_api):
        self.ble_api = ble_api
        self.is_playing = False
        self.CHAIN_JUMP_INDEX = 16
        self.active_actuators = set()
        self.last_sent_commands = [] # for the end of the slider use


    def detect_leaving_edges(self, current_amplitudes):
        current_actuators = set(current_amplitudes.keys())
        leaving_edges = self.active_actuators - current_actuators
        stop_commands = [
            self.prepare_command(actuator_id, 0, 0, 0)  # Prepare stop command
            for actuator_id in leaving_edges
        ]
        self.active_actuators = current_actuators
        return stop_commands

    def actuator_id_to_addr(self, actuator_id):
        chain, index = actuator_id.split('.')
        return (ord(chain) - ord('A')) * self.CHAIN_JUMP_INDEX + int(index) - 1

    def map_amplitude_to_duty(self, amplitude):
        # Map amplitude from 0 to 1 to 0 to 15
        return int(round(amplitude * 15))


    def map_frequency_to_freq_param(self, frequency):
        # Define the frequency set
        frequency_set = [123, 145, 170, 200, 235, 275, 322, 384]
        
        # Find the closest frequency in the set
        closest_freq = min(frequency_set, key=lambda x: abs(x - frequency))
        
        # Return the index of the closest frequency
        return frequency_set.index(closest_freq)

    def prepare_command(self, actuator_id, amplitude, frequency, start_or_stop=1):
        return {
            'addr': self.actuator_id_to_addr(actuator_id),
            'duty': self.map_amplitude_to_duty(amplitude),
            'freq': self.map_frequency_to_freq_param(frequency),
            'start_or_stop': start_or_stop
        }  # 1 for start

    def process_commands(self, commands):
        commands.sort(key=lambda cmd: cmd["addr"])
        if commands != None and self.is_playing:
            self.ble_api.send_command_list(commands)  # Send the list of commands
            self.last_sent_commands = commands  # Log the last sent commands
            print(f"Sending command list at Time {time.perf_counter()}: {commands}")

    def start_playback(self):
        self.is_playing = True

    def stop_playback(self):
        self.is_playing = False
        # Generate stop commands for all active actuators based on the current active signals
        stop_commands = [
            {"addr": self.actuator_id_to_addr(actuator_id), "duty": 0, "freq": 0, "start_or_stop": 0}
            for actuator_id in self.active_actuators
        ]
        # sort the stop_commands by addr in a increasing order
        stop_commands.sort(key=lambda cmd: cmd["addr"])
        
        # Send STOP commands to the actuators
        if stop_commands:
            self.ble_api.send_command_list(stop_commands)  # Send the list of stop commands
            self.last_sent_commands = stop_commands  # Log the last sent stop commands
            current_time = time.time()
            print(f"[Play Button Stopping] Sending stop command list at {current_time}: {stop_commands}")
        
        # Clear active actuators and active signals after sending the stop commands
        self.active_actuators.clear()


    def update(self, current_amplitudes):
        """Update the playing signals if there is a change."""
        # Detect leaving edges and generate stop commands
        stop_commands = self.detect_leaving_edges(current_amplitudes)

        # Prepare all commands for active actuators, ensuring we handle None parameters safely
        active_commands = []
        for actuator_id, signal_details in current_amplitudes.items():
            active_commands.append(self.prepare_command(actuator_id, signal_details["current_amplitude"], signal_details["current_frequency"], 1))

        all_commands = stop_commands + active_commands
        # Process the commands (not time-guarded)
        self.process_commands(all_commands)

class DesignSaver:
    def __init__(self, actuator_canvas, timeline_canvases, mpl_canvas, app_reference):
        self.actuator_canvas = actuator_canvas
        self.timeline_canvases = timeline_canvases
        self.mpl_canvas = mpl_canvas
        self.app_reference = app_reference

    def prompt_save_before_loading(self):
        response = QMessageBox.question(
            None,
            "Save Current Design",
            "Do you want to save the current design before loading a new one?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No | QMessageBox.StandardButton.Cancel
        )
        
        if response == QMessageBox.StandardButton.Yes:
            self.save_design()
            return True
        elif response == QMessageBox.StandardButton.No:
            return True
        else:
            return False

    def save_design(self):
        file_name, _ = QFileDialog.getSaveFileName(None, "Save Design As", "", "Design Files (*.dsgn)")
        if file_name:
            try:
                design_data = {
                    'actuators': self.collect_actuator_data(),
                    'timeline': self.collect_timeline_data(),
                    'imported_signals': self.app_reference.imported_signals,
                    'custom_signals': self.app_reference.custom_signals,
                    'branch_colors': self.actuator_canvas.branch_colors,
                    'mpl_canvas_data': self.collect_mpl_canvas_data(),
                    'current_actuator': self.app_reference.current_actuator,
                    'actuator_signals': self.app_reference.actuator_signals,
                    'tree_widget_data': self.collect_tree_widget_data(),
                }

                with open(file_name, 'wb') as file:
                    pickle.dump(design_data, file)
                QMessageBox.information(None, "Success", "Design saved successfully!")
            except Exception as e:
                QMessageBox.warning(None, "Error", f"Failed to save design: {str(e)}")

    def load_design(self):
        if not self.prompt_save_before_loading():
            return

        file_name, _ = QFileDialog.getOpenFileName(None, "Open Design", "", "Design Files (*.dsgn)")
        if file_name:
            try:
                with open(file_name, 'rb') as file:
                    design_data = pickle.load(file)

                self.app_reference.clear_canvas_and_timeline(bypass_dialog=True)
                
                self.apply_actuator_data(design_data['actuators'])
                self.apply_timeline_data(design_data['timeline'])
                self.app_reference.imported_signals = design_data.get('imported_signals', {})
                self.app_reference.custom_signals = design_data.get('custom_signals', {})
                self.actuator_canvas.branch_colors = design_data.get('branch_colors', {})
                self.apply_mpl_canvas_data(design_data.get('mpl_canvas_data', {}))
                self.app_reference.current_actuator = design_data.get('current_actuator')
                self.app_reference.actuator_signals = design_data.get('actuator_signals', {})
                self.apply_tree_widget_data(design_data.get('tree_widget_data', {}))

                self.actuator_canvas.redraw_all_lines()
                
                if self.app_reference.current_actuator:
                    self.app_reference.switch_to_timeline_canvas(self.app_reference.current_actuator)
                else:
                    self.app_reference.switch_to_main_canvas()

                QMessageBox.information(None, "Success", "Design loaded successfully!")
                self.app_reference.update_actuator_text()
                self.app_reference.update_pushButton_5_state()
            except Exception as e:
                QMessageBox.warning(None, "Error", f"Failed to load design: {str(e)}")

    def collect_tree_widget_data(self):
        tree_data = {}
        root = self.app_reference.ui.treeWidget.invisibleRootItem()
        for i in range(root.childCount()):
            top_level_item = root.child(i)
            if top_level_item.text(0) in ["Imported Signals", "Customized Signals"]:
                tree_data[top_level_item.text(0)] = self.collect_child_items(top_level_item)
        return tree_data
    
    def collect_child_items(self, parent_item):
        items = []
        for i in range(parent_item.childCount()):
            child = parent_item.child(i)
            items.append({
                'text': child.text(0),
                'tooltip': child.toolTip(0),
                'user_data': child.data(0, QtCore.Qt.ItemDataRole.UserRole)
            })
        return items

    def apply_tree_widget_data(self, tree_data):
        root = self.app_reference.ui.treeWidget.invisibleRootItem()
        
        for category in ["Imported Signals", "Customized Signals"]:
            category_item = None
            for i in range(root.childCount()):
                top_level_item = root.child(i)
                if top_level_item.text(0) == category:
                    category_item = top_level_item
                    break
            
            if category_item is None:
                category_item = QTreeWidgetItem(self.app_reference.ui.treeWidget)
                category_item.setText(0, category)

            category_item.takeChildren()  # Clear existing children
            
            for item_data in tree_data.get(category, []):
                child = QTreeWidgetItem(category_item)
                child.setText(0, item_data['text'])
                child.setToolTip(0, item_data['tooltip'])
                child.setData(0, QtCore.Qt.ItemDataRole.UserRole, item_data['user_data'])
                child.setFlags(child.flags() | QtCore.Qt.ItemFlag.ItemIsEditable)

            self.app_reference.ui.treeWidget.expandItem(category_item)

    def collect_actuator_data(self):
        return [{
            'id': actuator.id,
            'type': actuator.actuator_type,
            'color': actuator.color.name(),
            'position': (actuator.pos().x(), actuator.pos().y()),
            'predecessor': actuator.predecessor,
            'successor': actuator.successor
        } for actuator in self.actuator_canvas.actuators]

    def collect_timeline_data(self):
        timeline_data = []
        for actuator_id, timeline_canvas in self.timeline_canvases.items():
            timeline_data.extend([{
                'actuator_id': actuator_id,
                'type': signal["type"],
                'start_time': signal["start_time"],
                'stop_time': signal["stop_time"],
                'data': signal["data"],  # Original signal data
                'high_freq': signal.get("high_freq", None),  # high frequency data
                'low_freq': signal.get("low_freq", None),    # low frequency data
                'parameters': signal["parameters"]
            } for signal in timeline_canvas.signals])
        return timeline_data


    def collect_mpl_canvas_data(self):
        return {
            'current_signal': self.mpl_canvas.current_signal.tolist() if self.mpl_canvas.current_signal is not None else None,
        }

    def apply_actuator_data(self, actuator_data):
        self.actuator_canvas.clear_canvas()
        for actuator_info in actuator_data:
            x, y = actuator_info['position']
            self.actuator_canvas.add_actuator(
                x, y, 
                new_id=actuator_info['id'], 
                actuator_type=actuator_info['type'], 
                predecessor=actuator_info['predecessor'], 
                successor=actuator_info['successor']
            )

    def apply_timeline_data(self, timeline_data):
        self.app_reference.actuator_signals.clear()
        for signal_info in timeline_data:
            actuator_id = signal_info['actuator_id']
            if actuator_id not in self.app_reference.actuator_signals:
                self.app_reference.actuator_signals[actuator_id] = []
            
            # Load signal data, including high and low frequency components
            self.app_reference.actuator_signals[actuator_id].append({
                'type': signal_info['type'],
                'start_time': signal_info['start_time'],
                'stop_time': signal_info['stop_time'],
                'data': signal_info['data'],  # Original data
                'high_freq': signal_info.get('high_freq', None),  # High frequency data
                'low_freq': signal_info.get('low_freq', None),    # Low frequency data
                'parameters': signal_info['parameters']
            })

        # Apply the signals to the timeline canvases
        for actuator_id, signals in self.app_reference.actuator_signals.items():
            if actuator_id in self.timeline_canvases:
                self.timeline_canvases[actuator_id].signals = signals
                self.timeline_canvases[actuator_id].plot_all_signals()

                

    def apply_mpl_canvas_data(self, mpl_data):
        if mpl_data['current_signal']:
            self.mpl_canvas.current_signal = np.array(mpl_data['current_signal'])
            self.mpl_canvas.plot(np.linspace(0, 1, len(self.mpl_canvas.current_signal)), self.mpl_canvas.current_signal)
        else:
            self.mpl_canvas.clear_plot()
            
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=8, height=2, dpi=100, app_reference=None):
        self.app_reference = app_reference  # Reference to Haptics_App
        self.current_signal = None  # Track the current signal
        self.current_signal_sampling_rate = None
        bg_color = (134/255, 150/255, 167/255)
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor=bg_color)
        self.axes = self.fig.add_axes([0.1, 0.15, 0.8, 0.8])  # Use add_axes to create a single plot
        self.axes.set_facecolor(bg_color)

        # Convert RGB to rgba using matplotlib.colors.to_rgba
        spine_color = to_rgba((240/255, 235/255, 229/255))

        self.axes.tick_params(axis='x', colors=spine_color, labelsize=8)  # Adjust tick label size here
        self.axes.tick_params(axis='y', colors=spine_color, labelsize=8)  # Adjust tick label size here
        self.axes.spines['bottom'].set_color(spine_color)
        self.axes.spines['top'].set_color(spine_color)
        self.axes.spines['right'].set_color(spine_color)
        self.axes.spines['left'].set_color(spine_color)
        self.axes.set_ylabel('Amplitude', fontsize=9.5, color=spine_color)  # Adjust font size here

        # Move x-axis label to the right side
        self.set_custom_xlabel('Time (s)', fontsize=9.5, color=spine_color)  # Custom method for xlabel

        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.setStyleSheet("background-color: rgb(134, 150, 167); color: spine_color;")
        self.fig.tight_layout()

        # Enable drag and drop
        self.setAcceptDrops(True)

        # Draw initial empty plot
        self.plot([], [])

    def set_custom_xlabel(self, xlabel, fontsize=9.5, color='black'):
        self.axes.set_xlabel('')  # Remove default xlabel
        self.axes.annotate(xlabel, xy=(1.01, -0.01), xycoords='axes fraction', fontsize=fontsize,
                           color=color, ha='left', va='center')

    def mousePressEvent(self, event):
        pass  # Disable mouse press event handling

    def plot(self, x, y):
        self.axes.clear()
        bg_color = (134/255, 150/255, 167/255)
        # Convert RGB to rgba using matplotlib.colors.to_rgba
        spine_color = to_rgba((240/255, 235/255, 229/255))
        self.axes.set_facecolor(bg_color)
        
        if len(x) == len(y):
            self.axes.plot(x, y, color=spine_color)
        else:
            print(f"Error: x and y must have the same length, but have shapes {x.shape} and {y.shape}")
        
        # Set x and y labels
        self.set_custom_xlabel('Time (s)', fontsize=9.5, color=spine_color)  # Custom method for xlabel
        self.axes.set_ylabel('Amplitude', fontsize=9.5, color=spine_color)  # Adjust font size here
        
        self.axes.tick_params(axis='x', colors=spine_color, labelsize=8)  # Adjust tick label size here
        self.axes.tick_params(axis='y', colors=spine_color, labelsize=8)  # Adjust tick label size here
        self.axes.set_ylim(-1.1, 1.1)  # Set y-axis limits
        self.axes.spines['bottom'].set_color(spine_color)
        self.axes.spines['top'].set_color(spine_color)
        self.axes.spines['right'].set_color(spine_color)
        self.axes.spines['left'].set_color(spine_color)
        self.draw()

    def add_signal(self, signal_data, combine):
        new_signal = np.array(signal_data["data"])
        new_signal_sampling_rate = signal_data["value0"]["sampling_rate"]
        
        if self.current_signal is None:
            self.current_signal = new_signal
            overall_t = len(self.current_signal)/TIME_STAMP

            
        else:
            # compare lengths and repeat the shorter one
            current_signal_length = len(self.current_signal)
            new_signal_length = len(new_signal)
            
            if current_signal_length > new_signal_length:
                repeat_factor = current_signal_length // new_signal_length + 1
                new_signal = np.tile(new_signal, repeat_factor)[:current_signal_length]
            elif current_signal_length < new_signal_length:
                repeat_factor = new_signal_length // current_signal_length + 1
                self.current_signal = np.tile(self.current_signal, repeat_factor)[:new_signal_length]

            # Combine or replace the signals
            if combine:
                self.current_signal = self.current_signal * new_signal
            else:
                self.current_signal = new_signal
            
            # Calculate the total time and adjust for plotting
            overall_t = len(self.current_signal) / TIME_STAMP
 
        t = np.linspace(0, overall_t, int(overall_t * TIME_STAMP))  # Generate t based on TIME_STAMP

        # Plot the resampled signal
        self.plot(t, self.current_signal)


    def clear_plot(self):
        self.current_signal = None
        self.plot([], [])

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-qabstractitemmodeldatalist'):
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        item = event.source().selectedItems()[0]
        signal_type = item.text(0)

        # Initialize customized_signal to None
        customized_signal = None

        # Check if the dropped signal is an imported one
        if signal_type in self.app_reference.imported_signals:
            customized_signal = self.app_reference.imported_signals[signal_type]
            self.add_signal(customized_signal, combine=True)

        # Check if the signal is a customized one
        elif signal_type in self.app_reference.custom_signals:
            customized_signal = self.app_reference.custom_signals[signal_type]
            self.add_signal(customized_signal, combine=True)

        else:
            parameters = {}  # Dictionary to store parameters

            # Handle oscillators
            if signal_type in ["Sine", "Square", "Saw", "Triangle"]:
                dialog = OscillatorDialog(signal_type, self)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    config = dialog.get_config()
                    parameters = config  # Store the parameters
                    customized_signal = self.generate_custom_general_oscillator_json(signal_type, config["frequency"], config["amplitude"], config["duration"])
                    self.app_reference.update_status_bar(signal_type, parameters)  # Update the status bar

            if signal_type in ["Chirp"]:
                dialog = ChirpDialog(signal_type, self)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    config = dialog.get_config()
                    parameters = config  # Store the parameters
                    customized_signal = self.generate_custom_chirp_json(signal_type, config["chirp_type"], config["frequency"], config['amplitude'], config["rate"], config["duration"])
                    self.app_reference.update_status_bar(signal_type, parameters)  # Update the status bar

            if signal_type in ["Noise"]:
                dialog = NoiseDialog(signal_type, self)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    config = dialog.get_config()
                    parameters = config  # Store the parameters
                    customized_signal = self.generate_custom_noise_json(signal_type, config["amplitude"],config["duration"])
                    self.app_reference.update_status_bar(signal_type, parameters)  # Update the status bar

            if signal_type in ["FM"]:
                dialog = FMDialog(signal_type, self)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    config = dialog.get_config()
                    parameters = config  # Store the parameters
                    customized_signal = self.generate_custom_FM_json(signal_type, config["FM_type"], config["frequency"], config['amplitude'], config["modulation"],config["index"],config["duration"])
                    self.app_reference.update_status_bar(signal_type, parameters)  # Update the status bar

            if signal_type in ["PWM"]:
                dialog = PWMDialog(signal_type, self)
                if dialog.exec() == QDialog.DialogCode.Accepted:
                    config = dialog.get_config()
                    parameters = config  # Store the parameters
                    customized_signal = self.generate_custom_PWM_json(signal_type, config["frequency"], config["amplitude"], config["duty_cycle"], config["duration"])
                    self.app_reference.update_status_bar(signal_type, parameters)  # Update the status bar


            # If a customized signal was created or retrieved, add it to the plot
            if customized_signal:
                self.add_signal(customized_signal, combine=True)

    def generate_custom_general_oscillator_json(self, signal_type, frequency, amplitude, duration):
        t = np.linspace(0, duration, int(TIME_STAMP * duration))
        if signal_type == "Sine":
            data = np.sin(2 * np.pi * frequency * t)
        elif signal_type == "Square":
            data = np.sign(np.sin(2 * np.pi * frequency * t))
        elif signal_type == "Saw":
            data = (2 * (t * frequency - np.floor(t * frequency + 0.5)))
        elif signal_type == "Triangle":
            data = (2 * np.abs(2 * (t * frequency - np.floor(t * frequency + 0.5))) - 1)
        else:
            data = np.zeros_like(t)  # Default for unsupported types
        data = (amplitude * data).tolist()
        return self.formatting_data(signal_type, data)

    def generate_custom_chirp_json(self, signal_type, chirp_type, frequency, amplitude, rate, duration):
        # Time array
        t = np.linspace(0, duration, int(TIME_STAMP * duration))
        
        # Calculate the frequency at each time point
        instantaneous_frequency = frequency + rate * t
        
        if chirp_type == 'Sine':
            # Generate the sine waveform with time-varying frequency
            data = np.sin(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
        elif chirp_type == 'Square':
            # Generate the square waveform with time-varying frequency
            data = signal.square(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
        elif chirp_type == 'Saw':
            # Generate the sawtooth waveform with time-varying frequency
            data = signal.sawtooth(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
        elif chirp_type == 'Triangle':
            # Generate the triangle waveform with time-varying frequency
            data = signal.sawtooth(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP, 0.5)
        else:
            data = np.zeros_like(t)  # Default for unsupported types
        data = (amplitude * data).tolist()
        return self.formatting_data(signal_type, data)
    
    def generate_custom_noise_json(self, signal_type, amplitude, duration):
        t = np.linspace(0, duration, int(TIME_STAMP * duration))
        # generate random noise between -1 and 1
        data = np.random.uniform(-1, 1, len(t))
        data = (amplitude * data).tolist()
        return self.formatting_data(signal_type, data)

    def generate_custom_FM_json(self, signal_type, FM_type, frequency, amplitude, modulation, index, duration):
        # Time array
        t = np.linspace(0, duration, int(TIME_STAMP * duration))
        
        # Instantaneous phase
        instantaneous_phase = 2 * np.pi * frequency * t + index * np.sin(2 * np.pi * modulation * t)
        
        if FM_type == 'Sine':
            # Generate the sine FM signal
            data = np.sin(instantaneous_phase)
        elif FM_type == 'Square':
            # Generate the square FM signal
            data = signal.square(instantaneous_phase)
        elif FM_type == 'Saw':
            # Generate the sawtooth FM signal
            data = signal.sawtooth(instantaneous_phase)
        elif FM_type == 'Triangle':
            # Generate the triangle FM signal
            data = signal.sawtooth(instantaneous_phase, 0.5)
        else:
            data = np.zeros_like(t)  # Default for unsupported types
        data = (amplitude * data).tolist()
        return self.formatting_data(signal_type, data)
        
    def generate_custom_PWM_json(self, signal_type, frequency, amplitude, duty_cycle, duration):
        # Time array
        t = np.linspace(0, duration, int(TIME_STAMP * duration))
        # generate PWM signal
        data = signal.square(2 * np.pi * frequency * t, duty_cycle/100)
        data = (amplitude * data).tolist()
        return self.formatting_data(signal_type, data)
    
    def formatting_data(self, signal_type, data):
        return {
            "value0": {
                "gain": 1.0,
                "bias": 0.0,
                "sampling_rate": TIME_STAMP,
                "m_ptr": {
                    "polymorphic_id": 2147483649,
                    "polymorphic_name": f"tact::Signal::Model<tact::{signal_type}>",
                    "ptr_wrapper": {
                        "valid": 1,
                        "data": {
                            "Concept": {},
                            "m_model": {
                                "IOscillator": {
                                    "x": {
                                        "gain": 0.0,
                                        "bias": 0.0,
                                        "m_ptr": {
                                            "polymorphic_id": 2147483650,
                                            "polymorphic_name": "tact::Signal::Model<tact::Time>",
                                            "ptr_wrapper": {
                                                "valid": 1,
                                                "data": {
                                                    "Concept": {},
                                                    "m_model": {}
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "data": data
        }

class PreviewCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=1, dpi=100, app_reference=None):
        self.app_reference = app_reference  # Reference to Haptics_App
        bg_color = (134/255, 150/255, 167/255)  # Same background color as MplCanvas
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor=bg_color)
        self.axes = self.fig.add_axes([0.1, 0.1, 0.8, 0.8])  # Use add_axes to create a single plot
        self.axes.set_facecolor(bg_color)

        # Convert RGB to rgba using matplotlib.colors.to_rgba
        spine_color = to_rgba((240/255, 235/255, 229/255))

        # Hide x-axis and y-axis labels, ticks, and tick labels
        self.axes.xaxis.set_visible(False)
        self.axes.yaxis.set_visible(False)

        # Set spine colors (canvas border)
        spine_color = to_rgba((240/255, 235/255, 229/255))
        for spine in self.axes.spines.values():
            spine.set_color(spine_color)
            spine.set_linewidth(1)  # Set border width

        # # Move x-axis label to the right side
        # self.set_custom_xlabel('Time (s)', fontsize=9.5, color=spine_color)  # Custom method for xlabel

        super(PreviewCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.setStyleSheet("background-color: rgb(134, 150, 167); color: spine_color;")
        self.fig.tight_layout()

        # Initialize the canvas size to match layout.ui
        # self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)

    def plot_default_signal(self, signal_data):
        self.axes.clear()

        # Define the color for the signal line
        spine_color = to_rgba((240/255, 235/255, 229/255))

        if signal_data is not None and "data" in signal_data:
            t = np.linspace(0, 1, len(signal_data["data"]))
            self.axes.plot(t, signal_data["data"], color=spine_color)  # Use spine_color for the line
        else:
            # Clear the plot if signal_data is None or invalid
            self.axes.clear()
        self.draw()

    def mousePressEvent(self, event):
        pass  # Disable mouse press event handling

    def dragEnterEvent(self, event):
        event.ignore()  # Disable drag event

    def dropEvent(self, event):
        event.ignore()  # Disable drop event

class ActuatorSignalHandler(QObject):
    clicked = pyqtSignal(str)  # Signal to indicate actuator is clicked
    properties_changed = pyqtSignal(str, str, str)  # Signal to indicate properties change: id, type, color


    def __init__(self, actuator_id, parent=None):
        super().__init__(parent)
        self.actuator_id = actuator_id

class Actuator(QGraphicsItem):
    def __init__(self, x, y, size, color, actuator_type, id, predecessor=None, successor=None):
        super().__init__()
        self.setPos(x, y)
        self.size = size
        self.color = color
        self.actuator_type = actuator_type
        self.id = id
        self.predecessor = predecessor
        self.successor = successor
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setAcceptHoverEvents(True)

        # Create a signal handler for this actuator
        self.signal_handler = ActuatorSignalHandler(self.id)

        # Apply configuration
        config = ACTUATOR_CONFIG.get(self.actuator_type, ACTUATOR_CONFIG["LRA"])
        self.text_vertical_offset = config["text_vertical_offset"]
        self.text_horizontal_offset = config["text_horizontal_offset"]
        self.font_size_factor = config["font_size_factor"]
        self.min_font_size = config["min_font_size"]
        self.max_font_size = config["max_font_size"]

        # Calculate initial font size
        self.font_size = self.calculate_font_size()   

    def calculate_font_size(self):
        base_size = self.size / 2 * self.font_size_factor
        id_length = len(self.id)
        if id_length > 3:
            base_size *= 3 / id_length
        return max(self.min_font_size, min(base_size, self.max_font_size))

    def get_color_name(self, color):
        colors = [
            (QColor(255, 0, 0), "Red"),
            (QColor(0, 255, 0), "Green"),
            (QColor(0, 0, 255), "Blue"),
            (QColor(255, 255, 0), "Yellow"),
            (QColor(255, 0, 255), "Magenta"),
            (QColor(0, 255, 255), "Cyan"),
            (QColor(225,127,147), "bbPink")
        ]
        for qcolor, name in colors:
            if qcolor == color:
                return name
        return "Unknown"

    def boundingRect(self):
        return QRectF(-self.size/2, -self.size/2, self.size, self.size)

    def paint(self, painter, option, widget):
        # Clear any previous drawing to avoid overlap
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(Qt.GlobalColor.black, 1))

        # Draw the actuator shape based on its type (without the highlight)
        if self.actuator_type == "LRA":
            painter.drawEllipse(self.boundingRect())
        elif self.actuator_type == "VCA":
            painter.drawRect(self.boundingRect())
        else:  # "M  "
            painter.drawRoundedRect(self.boundingRect(), 5, 5)

        # Now, only draw the highlight rim if the item is selected
        if self.isSelected():
            highlight_pen = QPen(QColor(225, 20, 146), 3)  # Thicker rim for highlighting
            painter.setPen(highlight_pen)
            
            # Draw the highlight rim (slightly larger than the original shape)
            if self.actuator_type == "LRA":
                painter.drawEllipse(self.boundingRect().adjusted(-2, -2, 2, 2))  # Slightly larger for the rim
            elif self.actuator_type == "VCA":
                painter.drawRect(self.boundingRect().adjusted(-2, -2, 2, 2))
            else:  # "M  "
                painter.drawRoundedRect(self.boundingRect().adjusted(-2, -2, 2, 2), 5, 5)

        # Set font size
        font = painter.font()
        font.setPointSizeF(self.calculate_font_size())
        painter.setFont(font)

        # Convert the ID to the desired format
        if '.' in self.id:
            main_id, sub_id = self.id.split('.')
            formatted_id = main_id + to_subscript(sub_id)
        else:
            formatted_id = self.id  # Handle cases where ID does not contain a '.'

        # Calculate text position
        rect = self.boundingRect()
        text_rect = QRectF(rect.left() + self.text_horizontal_offset,
                        rect.top() + self.text_vertical_offset,
                        rect.width(),
                        rect.height())

        # Draw text
        painter.drawText(text_rect, Qt.AlignmentFlag.AlignCenter, formatted_id)


    def hoverEnterEvent(self, event):
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def hoverLeaveEvent(self, event):
        self.setCursor(Qt.CursorShape.ArrowCursor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            self.signal_handler.clicked.emit(self.id)  # Emit the signal with the actuator's ID
        super().mousePressEvent(event)
        
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.setCursor(Qt.CursorShape.OpenHandCursor)
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

        # Get the ActuatorCanvas view and its canvas rectangle
        canvas = self.scene().views()[0]
        canvas_rect = canvas.canvas_rect

        # Calculate the bounding box of all selected items
        selected_items = self.scene().selectedItems()
        if not selected_items:
            return

        bounding_rect = QRectF()
        for item in selected_items:
            bounding_rect = bounding_rect.united(item.mapRectToScene(item.boundingRect()))

        # Calculate the delta movement from the original position
        delta = event.scenePos() - event.lastScenePos()

        # Check if the bounding box after movement will be within the canvas limits
        if bounding_rect.left() + delta.x() < canvas_rect.left():
            delta.setX(canvas_rect.left() - bounding_rect.left())
        if bounding_rect.right() + delta.x() > canvas_rect.right():
            delta.setX(canvas_rect.right() - bounding_rect.right())
        if bounding_rect.top() + delta.y() < canvas_rect.top():
            delta.setY(canvas_rect.top() - bounding_rect.top())
        if bounding_rect.bottom() + delta.y() > canvas_rect.bottom():
            delta.setY(canvas_rect.bottom() - bounding_rect.bottom())

        # Apply the adjusted movement to all selected items
        for item in selected_items:
            item.moveBy(delta.x(), delta.y())

        # Trigger a full repaint of the canvas during dragging
        self.update()  # Force repaint of the canvas
        canvas.update()  # Ensure the view is updated

        # Trigger a redraw of all lines
        canvas.redraw_all_lines()

    def adjust_text_position(self, vertical_offset, horizontal_offset):
        self.text_vertical_offset = vertical_offset
        self.text_horizontal_offset = horizontal_offset
        self.update()

    def adjust_font_size(self, factor, min_size, max_size):
        self.font_size_factor = factor
        self.min_font_size = min_size
        self.max_font_size = max_size
        self.update()
    
    def update_properties(self, actuator_type, color):
        self.actuator_type = actuator_type
        self.color = color
        self.update()
        self.signal_handler.properties_changed.emit(self.id, self.actuator_type, self.color.name())

class ActuatorPropertiesDialog(QDialog):
    def __init__(self, actuator, parent=None):
        super().__init__(parent)
        self.actuator = actuator
        self.setWindowTitle("Actuator Properties")
        self.layout = QVBoxLayout(self)

        form_layout = QFormLayout()
        self.id_input = QLineEdit(actuator.id)
        form_layout.addRow("ID:", self.id_input)

        type_layout = QHBoxLayout()
        self.type_group = QButtonGroup(self)
        self.lra_radio = QRadioButton("LRA")
        self.vca_radio = QRadioButton("VCA")
        self.m_radio = QRadioButton("M  ")
        self.type_group.addButton(self.lra_radio)
        self.type_group.addButton(self.vca_radio)
        self.type_group.addButton(self.m_radio)
        type_layout.addWidget(self.lra_radio)
        type_layout.addWidget(self.vca_radio)
        type_layout.addWidget(self.m_radio)
        form_layout.addRow("Type:", type_layout)

        self.predecessor_input = QLineEdit(actuator.predecessor or "")
        self.successor_input = QLineEdit(actuator.successor or "")
        form_layout.addRow("Predecessor:", self.predecessor_input)
        form_layout.addRow("Successor:", self.successor_input)

        self.layout.addLayout(form_layout)

        button = QPushButton("OK")
        button.clicked.connect(self.accept)
        self.layout.addWidget(button)

        self.set_initial_type()

    def set_initial_type(self):
        if self.actuator.actuator_type == "LRA":
            self.lra_radio.setChecked(True)
        elif self.actuator.actuator_type == "VCA":
            self.vca_radio.setChecked(True)
        else:
            self.m_radio.setChecked(True)

    def get_type(self):
        if self.lra_radio.isChecked():
            return "LRA"
        elif self.vca_radio.isChecked():
            return "VCA"
        else:
            return "M  "

class SelectionBar(QGraphicsItem):
    def __init__(self, scene, parent=None):
        super().__init__()
        self.setPos(10, 10)  # Default location in the top left corner
        self.setAcceptHoverEvents(True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.selection_icons = []
        self.scene = scene
        self.create_selection_icons()

    def create_selection_icons(self):
        actuator_types = ["LRA", "VCA", "M  "]
        for i, act_type in enumerate(actuator_types):
            icon = Actuator(0, 0, 20, QColor(240, 235, 229), act_type, act_type)
            icon.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, False)
            icon.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)
            icon.setPos(i*30 ,0)
            icon.setPos(-100, i * 30)
            icon.adjust_font_size(0.3, 6, 12)  # Adjust font size here
            self.selection_icons.append(icon)
            self.scene.addItem(icon)

class ActuatorCanvas(QGraphicsView):
    actuator_added = pyqtSignal(str, str, str, int, int)  # Signal to indicate an actuator is added with its properties
    properties_changed = pyqtSignal(str, str, str, str)
    actuator_deleted = pyqtSignal(str)  # Signal to indicate an actuator is deleted
    no_actuator_selected = pyqtSignal()

    def __init__(self, parent=None, app_reference=None):
        super().__init__(parent)
        self.haptics_app = app_reference  # Store the reference to the Haptics_App instance

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHints(QPainter.RenderHint.Antialiasing)

        self.actuators = []
        self.branch_colors = {}
        self.color_index = 0  # Index for the color list

        # Set background color to rgb(134, 150, 167)
        self.setBackgroundBrush(QBrush(QColor(134, 150, 167)))
        self.setSceneRect(0, 0, 700, 300)  # Large scene to allow panning
        
        self.canvas_rect = QRectF(0, 0, 700, 300)
        self.white_rect_item = None
        self.update_canvas_visuals()

        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        self.setAcceptDrops(True)  # Allow drop events

        self.panning = False
        self.last_pan_point = QPointF()
        self.actuator_size = 20

        self.scene.selectionChanged.connect(self.handle_selection_change)

    def handle_selection_change(self):
        # Force an update on the scene to repaint the actuators
        for item in self.scene.selectedItems():
            item.update()  # Ensure selected items are redrawn
        
        # Update the entire scene to clear any artifacts
        self.scene.update()


    def draw_arrowhead(self, line, ratio=0.55):
        # Calculate the midpoint of the line
        midpoint = QPointF((line.p1().x() + ratio * (line.p2().x() - line.p1().x())), 
                            (line.p1().y() + ratio * (line.p2().y() - line.p1().y())))

        # Size of the arrowhead
        arrow_size = 5

        # Create a simple triangle for the arrowhead
        arrow_head = QPolygonF()
        arrow_head.append(QPointF(0, 0))  # The tip of the arrow
        arrow_head.append(QPointF(-arrow_size, arrow_size/2))
        arrow_head.append(QPointF(-arrow_size, -arrow_size/2))

        # Calculate the angle of the line
        angle = line.angle()

        # Rotate the arrowhead to match the direction of the line
        transform = QTransform()
        transform.translate(midpoint.x(), midpoint.y())  # Move the arrowhead to the midpoint
        transform.rotate(-angle)  # Rotate it according to the line's angle

        # Apply the transformation to the arrowhead
        arrow_head = transform.map(arrow_head)

        # Draw the arrowhead
        arrow_item = self.scene.addPolygon(arrow_head, QPen(Qt.GlobalColor.black), QBrush(Qt.GlobalColor.black))
        arrow_item.setZValue(-1)  # Ensure the arrowhead is behind the actuators

    def redraw_all_lines(self):
        """Redraw all lines connecting actuators and check for topology conflicts."""
        # Remove all existing lines and arrowheads except for the scale line and text
        for item in self.scene.items():
            if isinstance(item, (QGraphicsLineItem, QGraphicsPolygonItem)):
                self.scene.removeItem(item)

        # Iterate through all actuators and draw lines when both conditions are met
        for actuator in self.actuators:
            # Check for topology conflicts
            if actuator.predecessor:
                predecessor_actuator = self.get_actuator_by_id(actuator.predecessor)
                if predecessor_actuator:
                    # Check if the predecessor lists the current actuator as its successor
                    if predecessor_actuator.successor != actuator.id:
                        self.generate_topology_conflict_warning(actuator.id, predecessor_actuator.id)
                        continue  # Skip drawing the line if there's a conflict
                else:
                    # Predecessor does not exist, issue a warning
                    print(f"Warning: Actuator {actuator.id} references a non-existent predecessor {actuator.predecessor}.")
                    continue  # Skip drawing if predecessor is invalid

            # Check if actuator has a successor and if both conditions are satisfied
            if actuator.successor:
                successor_actuator = self.get_actuator_by_id(actuator.successor)
                if successor_actuator:
                    # Check if the successor lists the current actuator as its predecessor
                    if successor_actuator.predecessor != actuator.id:
                        self.generate_topology_conflict_warning(actuator.id, successor_actuator.id)
                        continue  # Skip drawing the line if there's a conflict
                else:
                    # Successor does not exist, issue a warning
                    print(f"Warning: Actuator {actuator.id} references a non-existent successor {actuator.successor}.")
                    continue  # Skip drawing if successor is invalid

            # Condition to draw the line: Check if both predecessor and successor match the requirements
            if actuator.predecessor:
                predecessor_actuator = self.get_actuator_by_id(actuator.predecessor)
                if predecessor_actuator and predecessor_actuator.successor == actuator.id:
                    # Draw the line from predecessor to current actuator
                    line = QLineF(predecessor_actuator.pos(), actuator.pos())
                    line_item = self.scene.addLine(line, QPen(Qt.GlobalColor.black, 2))
                    line_item.setZValue(-1)  # Ensure the line is behind the actuators
                    self.draw_arrowhead(line)

            # Draw the arrow connecting to the successor
            if actuator.successor:
                successor_actuator = self.get_actuator_by_id(actuator.successor)
                if successor_actuator and successor_actuator.predecessor == actuator.id:
                    line = QLineF(actuator.pos(), successor_actuator.pos())
                    line_item = self.scene.addLine(line, QPen(Qt.GlobalColor.black, 2))
                    line_item.setZValue(-1)  # Ensure the line is behind the actuators
                    self.draw_arrowhead(line)

    def generate_topology_conflict_warning(self, actuator_id_1, actuator_id_2):
        """Generate a warning message for a topology conflict between two actuators."""
        message = (
            f"Topology Conflict Detected!<br>"
            f"Actuator '<b>{actuator_id_1}</b>' claims its successor is '<b>{actuator_id_2}</b>', but "
            f"'<b>{actuator_id_2}</b>' does not recognize '<b>{actuator_id_1}</b>' as its predecessor.<br>"
            "Please check the configuration."
        )
        QMessageBox.warning(self, "Topology Conflict", message)

    def generate_same_predecessor_successor_warning(self, actuator_id):
        """Generate a warning message for an actuator with the same predecessor and successor."""
        message = (
            f"Topology Error Detected!<br>"
            f"Actuator '<b>{actuator_id}</b>' has the <b>same</b> predecessor and successor, which is not allowed.<br>"
            "Please check the configuration."
        )
        QMessageBox.warning(self, "Configuration Error", message)



    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()

    def dragMoveEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()

    def dropEvent(self, event):
        if event.mimeData().hasText():
            actuator_type = event.mimeData().text()
            pos = self.mapToScene(event.position().toPoint())
            
            # Check if the drop position is within the allowed "white area"
            if self.is_drop_allowed(pos):
                self.add_actuator(pos.x(), pos.y(), actuator_type=actuator_type)
                event.acceptProposedAction()
            else:
                event.ignore()

    def add_actuator(self, x, y, new_id=None, actuator_type="LRA", predecessor=None, successor=None):
        # Generate a new ID if not provided
        if new_id is None:
            new_id = self.generate_next_id()
        
        # Determine the branch (e.g., A for A.1)
        branch = new_id.split('.')[0]
        if branch not in self.branch_colors:
            if self.color_index < len(COLOR_LIST):
                self.branch_colors[branch] = COLOR_LIST[self.color_index]
                self.color_index += 1
            else:
                self.branch_colors[branch] = generate_contrasting_color(list(self.branch_colors.values()))

        color = self.branch_colors[branch]

        # Automatically determine predecessor if not provided
        if predecessor is None or successor is None:
            predecessor, successor = self.get_predecessor_successor(new_id)

        # Create and add the new actuator
        actuator = Actuator(x, y, self.actuator_size, color, actuator_type, new_id, predecessor, successor)
        self.scene.addItem(actuator)
        self.actuators.append(actuator)
        actuator.setZValue(0)  # Ensure actuator is above the lines

        # Update predecessor's successor to the newly added actuator
        if predecessor:
            pred_actuator = self.get_actuator_by_id(predecessor)
            if pred_actuator:
                pred_actuator.successor = new_id  # Update the predecessor's successor to the new actuator
                pred_actuator.update()

                # Draw a line from the predecessor to the newly added actuator
                line = QLineF(pred_actuator.pos(), actuator.pos())
                line_item = self.scene.addLine(line, QPen(Qt.GlobalColor.black, 2))
                line_item.setZValue(-1)  # Ensure the line is behind the actuators
                self.draw_arrowhead(line)  # Draw the arrowhead

        # Draw an arrow connecting to the successor (if applicable)
        if successor:
            for act in self.actuators:
                if act.id == successor:
                    line = QLineF(actuator.pos(), act.pos())
                    line_item = self.scene.addLine(line, QPen(Qt.GlobalColor.black, 2))
                    line_item.setZValue(-1)  # Ensure the line is behind the actuators
                    self.draw_arrowhead(line)
                    break

        actuator.update()  # Update the new actuator to reflect changes

        # Emit signal indicating the actuator has been added
        self.actuator_added.emit(new_id, actuator_type, color.name(), x, y)


    def get_actuator_by_id(self, actuator_id):
        """Retrieve an actuator by its ID."""
        for actuator in self.actuators:
            if actuator.id == actuator_id:
                return actuator
        return None

    def is_drop_allowed(self, pos):
        return self.canvas_rect.contains(pos)


    def update_canvas_visuals(self):
        # Remove the old white rectangle if it exists
        if self.white_rect_item:
            self.scene.removeItem(self.white_rect_item)
            self.white_rect_item = None

        # Add the new white rectangle
        self.white_rect_item = self.scene.addRect(self.canvas_rect, QPen(Qt.GlobalColor.black), QBrush(QColor(240, 235, 229)))
        self.white_rect_item.setZValue(-999)

    def mousePressEvent(self, event):
        item = self.itemAt(event.pos())

        if event.button() == Qt.MouseButton.LeftButton:
            if isinstance(item, Actuator):  # Left-click on an actuator
                self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
                self.setCursor(Qt.CursorShape.ClosedHandCursor)
                item.signal_handler.clicked.emit(item.id)  # Emit the signal with the actuator's ID from the Actuator
                super().mousePressEvent(event)
            else:
                self.setDragMode(QGraphicsView.DragMode.RubberBandDrag)
                self.no_actuator_selected.emit()  # Emit the signal when no actuator is selected
                super().mousePressEvent(event)

        elif event.button() == Qt.MouseButton.MiddleButton:  # Enable panning with middle mouse button
            self.panning = True
            self.last_pan_point = event.pos()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()

        elif event.button() == Qt.MouseButton.RightButton:  # Context menu for right-click
            if isinstance(item, Actuator):
                self.show_context_menu(item, event.pos())
            else:
                self.no_actuator_selected.emit()
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.panning:
            delta = event.pos() - self.last_pan_point
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            self.last_pan_point = event.pos()
            event.accept()
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.panning:
            self.panning = False
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
        elif hasattr(self, 'dragging_item') and self.dragging_item:
            if hasattr(self, 'dragging_actuator') and self.dragging_actuator:
                pos = self.mapToScene(event.pos())
                if not self.is_drop_allowed(pos):
                    self.scene.removeItem(self.dragging_actuator)
                    self.actuators.remove(self.dragging_actuator)
                self.dragging_actuator = None
            self.dragging_item = None
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def update_dragging_item(self, event):
        if hasattr(self, 'dragging_actuator') and self.dragging_actuator:
            pos = self.mapToScene(event.pos())
            self.dragging_actuator.setPos(pos.x(), pos.y())

    def start_dragging_item(self, event):
        item = self.dragging_item
        if item:
            actuator_type = item.actuator_type
            pos = self.mapToScene(event.pos())
            self.add_actuator(pos.x(), pos.y(), actuator_type=actuator_type)
            self.dragging_actuator = self.actuators[-1]  # Reference to the newly created actuator
            self.drag_start_pos = event.pos()  # Update drag start position

            # Example of using QPixmap if applicable
            pixmap = QPixmap(100, 100)  # Ensure pixmap is properly initialized
            if pixmap.isNull():
                pass
            else:
                scaled_pixmap = pixmap.scaled(50, 50, Qt.AspectRatioMode.KeepAspectRatio)
                drag = QDrag(self)
                mime_data = QMimeData()
                drag.setMimeData(mime_data)
                drag.setPixmap(scaled_pixmap)
                drag.exec()


    def wheelEvent(self, event: QWheelEvent):
        zoom_factor = 1.05  # Reduced from 1.25 to make zooming less sensitive

        old_pos = self.mapToScene(event.position().toPoint())

        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)

        new_pos = self.mapToScene(event.position().toPoint())

        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.canvas_rect, Qt.AspectRatioMode.KeepAspectRatio)

    def generate_next_id(self):
        if not self.actuators:
            return "A.1"
        
        max_branch = max(act.id.split('.')[0] for act in self.actuators if '.' in act.id)
        max_number = max(int(act.id.split('.')[1]) for act in self.actuators if '.' in act.id and act.id.startswith(max_branch))
        
        return f"{max_branch}.{max_number + 1}"

    def get_predecessor_successor(self, new_id):
        branch, number = new_id.split('.')
        number = int(number)
        
        predecessor = None
        for act in reversed(self.actuators):
            if '.' in act.id:
                act_branch, act_number = act.id.split('.')
                if act_branch == branch and int(act_number) == number - 1:
                    predecessor = act.id
                    break
        
        return predecessor, None

    def show_context_menu(self, actuator, pos):
        menu = QMenu()
        edit_action = menu.addAction("Edit Properties")
        delete_action = menu.addAction("Delete")
        clear_signal_action = menu.addAction("Clear Signal")

        action = menu.exec(self.mapToGlobal(pos))
        if action == edit_action:
            self.edit_actuator_properties(actuator)
        elif action == delete_action:
            selected_items = self.scene.selectedItems()
            if selected_items:
                for item in selected_items:
                    if isinstance(item, Actuator):
                        self.remove_actuator(item)
            else:
                self.remove_actuator(actuator)
        elif action == clear_signal_action:  # Handle clearing the signal
            self.clear_actuator_signal(actuator)
    
    def clear_actuator_signal(self, actuator):
        """Clear the corresponding signal for the actuator."""
        actuator_id = actuator.id
        if actuator_id in self.haptics_app.actuator_signals:
            del self.haptics_app.actuator_signals[actuator_id]  # Remove the signal from the dictionary

        # Update the pushButton_5 state to reflect the change in signals
        self.haptics_app.update_pushButton_5_state()

        # Immediately update the plotter by switching back to the main canvas
        self.haptics_app.switch_to_main_canvas()
        self.haptics_app.update_actuator_text()


    def edit_actuator_properties(self, actuator):
        dialog = ActuatorPropertiesDialog(actuator, self)
        while True:
            if dialog.exec() == QDialog.DialogCode.Accepted:
                old_id = actuator.id
                new_id = dialog.id_input.text()

                # Check for ID conflicts
                if any(act.id == new_id and act != actuator for act in self.actuators):
                    # Show a warning message if there's a conflict
                    QMessageBox.warning(self, "ID Conflict Detected", f"Actuator ID '{new_id}' already exists. Please choose a different ID.")
                    continue  # Reopen the dialog for the user to change the ID
                
                actuator.id = new_id
                
                # Update color if branch has changed
                old_branch = old_id.split('.')[0]
                new_branch = new_id.split('.')[0]
                if old_branch != new_branch:
                    if actuator.predecessor:
                        predecessor_actuator = self.get_actuator_by_id(actuator.predecessor)
                        if predecessor_actuator:
                            predecessor_actuator.successor = None
                        
                    if actuator.successor:
                        successor_actuator = self.get_actuator_by_id(actuator.successor)
                        if successor_actuator:
                            successor_actuator.predecessor = None

                    if new_branch not in self.branch_colors:
                        if self.color_index < len(COLOR_LIST):
                            self.branch_colors[new_branch] = COLOR_LIST[self.color_index]
                            self.color_index += 1
                        else:
                            self.branch_colors[new_branch] = generate_contrasting_color(list(self.branch_colors.values()))
                    actuator.color = self.branch_colors[new_branch]
                
                new_type = dialog.get_type()
                if new_type != actuator.actuator_type:
                    actuator.actuator_type = new_type
                    # Reapply configuration for new type
                    config = ACTUATOR_CONFIG.get(actuator.actuator_type, ACTUATOR_CONFIG["LRA"])
                    actuator.text_vertical_offset = config["text_vertical_offset"]
                    actuator.text_horizontal_offset = config["text_horizontal_offset"]
                    actuator.font_size_factor = config["font_size_factor"]
                    actuator.min_font_size = config["min_font_size"]
                    actuator.max_font_size = config["max_font_size"]
                
                actuator.predecessor = dialog.predecessor_input.text()
                actuator.successor = dialog.successor_input.text()
                
                actuator.size = self.actuator_size  # Use the canvas's actuator size
                
                if old_branch != new_branch:
                    actuator.predecessor = None
                    actuator.successor = None
                
                actuator.update()
                
                # Update other actuators if necessary
                self.update_related_actuators(old_id, new_id)

                self.properties_changed.emit(old_id, new_id, new_type, actuator.color.name())

                # Update plotter immediately
                self.haptics_app.update_timeline_actuator(old_id, new_id, new_type, actuator.color.name())

                self.redraw_all_lines()  # Trigger a redraw of all lines

                break  # Exit the loop if everything is fine

            else:
                break  # Exit if the dialog is canceled


    def update_related_actuators(self, old_id, new_id):
        for act in self.actuators:
            if act.predecessor == old_id:
                act.predecessor = new_id
            if act.successor == old_id:
                act.successor = new_id
            act.update()

    def remove_actuator(self, actuator):
        """Remove an actuator and update its predecessor and successor appropriately."""
        if actuator.predecessor or actuator.successor:
            predecessor_actuator = self.get_actuator_by_id(actuator.predecessor) if actuator.predecessor else None
            successor_actuator = self.get_actuator_by_id(actuator.successor) if actuator.successor else None

            # Handle the case of A-B-C, where B is being deleted
            if predecessor_actuator and successor_actuator:
                # Update A's successor to be C
                predecessor_actuator.successor = successor_actuator.id
                predecessor_actuator.update()

                # Update C's predecessor to be A
                successor_actuator.predecessor = predecessor_actuator.id
                successor_actuator.update()

            elif predecessor_actuator:
                # If there is only a predecessor (no successor), just remove the successor reference from A
                predecessor_actuator.successor = None
                predecessor_actuator.update()

            elif successor_actuator:
                # If there is only a successor (no predecessor), just remove the predecessor reference from C
                successor_actuator.predecessor = None
                successor_actuator.update()

        # Remove the actuator from the scene
        self.actuators.remove(actuator)
        self.scene.removeItem(actuator)
        self.actuator_deleted.emit(actuator.id)  # Emit the deletion signal

        # Redraw all lines after deletion
        self.redraw_all_lines()       
        self.haptics_app.update_pushButton_5_state()

    def set_canvas_size(self, width, height):
        self.canvas_rect = QRectF(0, 0, width, height)
        self.update_canvas_visuals()
        self.fitInView(self.canvas_rect, Qt.AspectRatioMode.KeepAspectRatio)

    def create_actuator_branch(self, num_actuators, lra_count, vca_count, m_count, grid_pattern):
        # Get the next branch letter
        if not self.actuators:
            next_branch = 'A'
        else:
            max_branch = max(act.id.split('.')[0] for act in self.actuators if '.' in act.id)
            next_branch = chr(ord(max_branch) + 1)

        # Parse grid pattern
        rows, cols = map(int, grid_pattern.split('x'))

        calculated_size = min(self.canvas_rect.width() / (cols + 1), self.canvas_rect.height() / (rows + 1)) * 0.6
        self.actuator_size = min(calculated_size, 20)

        # Calculate spacing
        spacing_x = self.canvas_rect.width() / (cols + 1)
        spacing_y = self.canvas_rect.height() / (rows + 1)

        # Create actuators
        actuator_types = ['LRA'] * lra_count + ['VCA'] * vca_count + ['M'] * m_count
        random.shuffle(actuator_types)

        for i in range(num_actuators):
            row = i // cols
            col = i % cols
            x = spacing_x * (col + 1)
            y = spacing_y * (row + 1)

            new_id = f"{next_branch}.{i+1}"
            actuator_type = actuator_types[i] if i < len(actuator_types) else 'LRA'

            predecessor = f"{next_branch}.{i}" if i > 0 else None
            successor = f"{next_branch}.{i+2}" if i < num_actuators - 1 else None

            self.add_actuator(x, y, new_id, actuator_type, predecessor, successor)

        self.actuator_size = 20
        
        for actuator in self.actuators:
            actuator.size = self.actuator_size
            actuator.update()

        self.update()
        self.haptics_app.update_pushButton_5_state() # Update play button 

    
    def clear_canvas(self):
        for actuator in self.actuators:
            self.scene.removeItem(actuator)
        self.actuators.clear()
        self.branch_colors.clear()
        self.actuator_size = 20  # Reset to default size
        self.update_canvas_visuals()
        self.haptics_app.update_pushButton_5_state() # Update play button

    def clear_lines_except_scale(self):
        # Remove all lines and arrows except the scale line and scale text
        for item in self.scene.items():
            if isinstance(item, QGraphicsLineItem):
                self.scene.removeItem(item)
            elif isinstance(item, QGraphicsPolygonItem):  # Assuming arrows are polygons
                self.scene.removeItem(item)
            elif isinstance(item, QGraphicsTextItem):
                self.scene.removeItem(item)

    def highlight_actuators_at_time(self, time_position):
        for actuator in self.actuators:
            signals = self.haptics_app.actuator_signals.get(actuator.id, [])
            is_active = any(signal["start_time"] <= time_position <= signal["stop_time"] for signal in signals)
            if is_active:
                actuator.setSelected(True)  # Highlight the actuator
            else:
                actuator.setSelected(False)  # Remove highlight
        self.update()  # Ensure the canvas is updated

class SelectionBarView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super().__init__(parent)
        self.setScene(scene)
        self.setRenderHints(QPainter.RenderHint.Antialiasing)
        self.setFixedSize(100, 100)  # Adjust size as needed
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setStyleSheet("background: transparent; border: none;")
        self.setMouseTracking(True)

    def mousePressEvent(self, event):
        item = self.itemAt(event.pos())
        if isinstance(item, Actuator):
            drag = QDrag(self)
            mime_data = QMimeData()
            mime_data.setText(item.actuator_type)
            drag.setMimeData(mime_data)
            drag.exec(Qt.DropAction.CopyAction)
        super().mousePressEvent(event)

class ActuatorPropertiesDialog(QDialog):
    def __init__(self, actuator, parent=None):
        super().__init__(parent)
        self.actuator = actuator
        self.setWindowTitle("Actuator Properties")
        self.layout = QVBoxLayout(self)

        form_layout = QFormLayout()
        self.id_input = QLineEdit(actuator.id)
        self.id_input.textChanged.connect(self.format_input)
        form_layout.addRow("ID:", self.id_input)

        type_layout = QHBoxLayout()
        self.type_group = QButtonGroup(self)
        self.lra_radio = QRadioButton("LRA")
        self.vca_radio = QRadioButton("VCA")
        self.m_radio = QRadioButton("M  ")
        self.type_group.addButton(self.lra_radio)
        self.type_group.addButton(self.vca_radio)
        self.type_group.addButton(self.m_radio)
        type_layout.addWidget(self.lra_radio)
        type_layout.addWidget(self.vca_radio)
        type_layout.addWidget(self.m_radio)
        form_layout.addRow("Type:", type_layout)

        self.predecessor_input = QLineEdit(actuator.predecessor or "")
        self.predecessor_input.textChanged.connect(self.format_input)
        form_layout.addRow("Predecessor:", self.predecessor_input)

        self.successor_input = QLineEdit(actuator.successor or "")
        self.successor_input.textChanged.connect(self.format_input)
        form_layout.addRow("Successor:", self.successor_input)

        self.layout.addLayout(form_layout)

        button = QPushButton("OK")
        button.clicked.connect(self.accept)
        self.layout.addWidget(button)

        self.set_initial_type()

    def set_initial_type(self):
        if self.actuator.actuator_type == "LRA":
            self.lra_radio.setChecked(True)
        elif self.actuator.actuator_type == "VCA":
            self.vca_radio.setChecked(True)
        else:
            self.m_radio.setChecked(True)

    def get_type(self):
        if self.lra_radio.isChecked():
            return "LRA"
        elif self.vca_radio.isChecked():
            return "VCA"
        else:
            return "M  "

    def format_input(self):
        sender = self.sender()
        cursor_position = sender.cursorPosition()
        text = sender.text()
        formatted_text = self.format_text(text)
        
        if formatted_text != text:
            # Check if a dot was added
            dot_added = ('.' in formatted_text) and ('.' not in text)
            
            sender.setText(formatted_text)
            
            # Adjust cursor position
            if dot_added and cursor_position > len(formatted_text.split('.')[0]):
                new_cursor_position = min(cursor_position + 1, len(formatted_text))
            else:
                new_cursor_position = min(cursor_position, len(formatted_text))
            
            sender.setCursorPosition(new_cursor_position)

    def format_text(self, text):
        # Remove any existing dots
        text = text.replace('.', '')
        
        # Split the text into letters and numbers
        letters = ''.join(filter(str.isalpha, text)).upper()
        numbers = ''.join(filter(str.isdigit, text))
        
        # Combine letters and numbers with a dot
        if letters and numbers:
            return f"{letters}.{numbers}"
        elif letters:
            return letters
        elif numbers:
            return f"A.{numbers}"
        else:
            return ""

class CreateBranchDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create Actuator Branch")
        layout = QVBoxLayout(self)

        self.num_actuators_input = QSpinBox()
        self.num_actuators_input.setMinimum(1)
        self.num_actuators_input.valueChanged.connect(self.update_max_counts)
        layout.addWidget(QLabel("Number of Actuators:"))
        layout.addWidget(self.num_actuators_input)

        self.lra_input = QSpinBox()
        self.lra_input.valueChanged.connect(self.check_total)
        layout.addWidget(QLabel("LRA Count:"))
        layout.addWidget(self.lra_input)

        self.vca_input = QSpinBox()
        self.vca_input.valueChanged.connect(self.check_total)
        layout.addWidget(QLabel("VCA Count:"))
        layout.addWidget(self.vca_input)

        self.m_input = QSpinBox()
        self.m_input.valueChanged.connect(self.check_total)
        layout.addWidget(QLabel("M Count:"))
        layout.addWidget(self.m_input)

        self.grid_pattern_input = QLineEdit()
        self.grid_pattern_input.textChanged.connect(self.validate_inputs)
        layout.addWidget(QLabel("Grid Pattern (e.g., 2x2, 3x3):"))
        layout.addWidget(self.grid_pattern_input)

        self.button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)

        self.num_actuators_input.setValue(1)
        self.update_max_counts()
        self.validate_inputs()

    def update_max_counts(self):
        total = self.num_actuators_input.value()
        self.lra_input.setMaximum(total)
        self.vca_input.setMaximum(total)
        self.m_input.setMaximum(total)
        self.check_total()
        self.validate_inputs()

    def check_total(self):
        total = self.num_actuators_input.value()
        sum_counts = self.lra_input.value() + self.vca_input.value() + self.m_input.value()
        
        if sum_counts > total:
            diff = sum_counts - total
            if self.sender() == self.lra_input:
                self.lra_input.setValue(max(0, self.lra_input.value() - diff))
            elif self.sender() == self.vca_input:
                self.vca_input.setValue(max(0, self.vca_input.value() - diff))
            elif self.sender() == self.m_input:
                self.m_input.setValue(max(0, self.m_input.value() - diff))
            
            # Recalculate sum_counts after adjustment
            sum_counts = self.lra_input.value() + self.vca_input.value() + self.m_input.value()

        self.validate_inputs()

    def accept(self):
        if (self.lra_input.value() + self.vca_input.value() + self.m_input.value() == self.num_actuators_input.value() and
            self.validate_grid_pattern(self.grid_pattern_input.text())):
            super().accept()

    def validate_grid_pattern(self, pattern):
        if not pattern.strip():  # Allow empty pattern
            return True
        try:
            rows, cols = map(int, pattern.split('x'))
            return rows > 0 and cols > 0  # Just check if it's a valid grid format
        except ValueError:
            return False
        
    def validate_inputs(self):
        total = self.num_actuators_input.value()
        sum_counts = self.lra_input.value() + self.vca_input.value() + self.m_input.value()
        grid_pattern = self.grid_pattern_input.text().strip()
        
        counts_valid = sum_counts == total
        grid_valid = self.validate_grid_pattern(grid_pattern)
        
        is_valid = counts_valid and grid_valid
        
        self.button_box.button(QDialogButtonBox.StandardButton.Ok).setEnabled(is_valid)

class TimelineCanvas(FigureCanvas):

    def __init__(self, parent=None, width=8, height=2, dpi=100, color=(134/255, 150/255, 167/255), label="", app_reference=None):
        self.app_reference = app_reference  # Reference to Haptics_App
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor=color)
        self.axes = self.fig.add_axes([0.1, 0.15, 0.8, 0.8])  # Use add_axes to create a single plot
        self.axes.set_facecolor(color)

        self.segmentation_api = signal_segmentation_api()
        
        # Set spine color and customize appearance
        spine_color = to_rgba((240/255, 235/255, 229/255))
        self.axes.spines['bottom'].set_color(spine_color)
        self.axes.spines['top'].set_color(spine_color)
        self.axes.spines['right'].set_color(spine_color)
        self.axes.spines['left'].set_color(spine_color)
        self.axes.tick_params(axis='x', colors=spine_color, labelsize=8)
        self.axes.tick_params(axis='y', colors=spine_color, labelsize=8)
        self.axes.set_ylabel('Amplitude', fontsize=9.5, color=spine_color)
        self.set_custom_xlabel('Time (s)', fontsize=9.5, color=spine_color)
        
        super(TimelineCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.setStyleSheet(f"background-color: rgba({int(color[0]*255)}, {int(color[1]*255)}, {int(color[2]*255)}, 0);")
        self.setAcceptDrops(True)
        
        self.signals = []  # List to store each signal's data along with their parameters

        # Variables to track dragging
        self._dragging = False
        self._last_mouse_x = None

        # dragggg
        self.signal_duration = 0  # Store the signal duration

    # dragggg
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.signal_duration > 2:
            self._dragging = True
            self._last_mouse_x = event.position().x()
    # dragggg
    def mouseMoveEvent(self, event):
        if self._dragging and self.signal_duration > 2:
            dx = event.position().x() - self._last_mouse_x
            self._last_mouse_x = event.position().x()
            xmin, xmax = self.axes.get_xlim()
            delta_x = dx * (xmax - xmin) / self.fig.get_size_inches()[0] / self.fig.dpi
            # Limit dragging to the signal duration
            if xmin - delta_x >= 0 and xmax - delta_x <= self.signal_duration:
                self.axes.set_xlim(xmin - delta_x, xmax - delta_x)
                self.draw()
    # dragggg
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._dragging = False

    def check_overlap(self, new_start_time, new_stop_time):
        for signal in self.signals:
            if not (new_stop_time <= signal["start_time"] or new_start_time >= signal["stop_time"]):
                return True
        return False

    def handle_overlap(self, new_start_time, new_stop_time, signal_type, signal_data, parameters):
        msg_box = QMessageBox(self)
        msg_box.setStyleSheet("background-color: rgb(193, 205, 215);")
        msg_box.setWindowTitle("Time Range Overlap")
        msg_box.setText(f"The time range overlaps with an existing signal.")
        msg_box.setInformativeText("Would you like to replace the overlapping region or reset the time range of the new signal?")

        replace_button = msg_box.addButton("Replace", QMessageBox.ButtonRole.YesRole)
        reset_button = msg_box.addButton("Reset", QMessageBox.ButtonRole.NoRole)

        msg_box.exec()

        if msg_box.clickedButton() == replace_button:
            self.replace_overlap(new_start_time, new_stop_time, signal_data, signal_type, parameters)
        else:
            # Adjust the previous signal to keep non-overlapping parts
            self.adjust_previous_signals(new_start_time, new_stop_time)

            # Prompt the user to set a new time range for the new signal
            start_time, stop_time = self.show_time_input_dialog(signal_type)
            if start_time is not None and stop_time is not None and stop_time > start_time:
                if self.check_overlap(start_time, stop_time):
                    self.handle_overlap(start_time, stop_time, signal_type, signal_data)
                else:
                    self.record_signal(signal_type, signal_data, start_time, stop_time, None)


    def replace_overlap(self, new_start_time, new_stop_time, new_signal_data, new_signal_type, new_signal_parameters):
        adjusted_signals = []

        for signal in self.signals:
            if signal["start_time"] < new_start_time < signal["stop_time"]:
                # Case: The new signal overlaps the end of this signal
                if new_stop_time < signal["stop_time"]:
                    # Trim the end of the original signal and keep the non-overlapping part
                    signal_part = {
                        "type": signal["type"],
                        "data": signal["data"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "high_freq": signal["high_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "low_freq": signal["low_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "start_time": signal["start_time"],
                        "stop_time": new_start_time,
                        "parameters": signal["parameters"]
                    }
                    adjusted_signals.append(signal_part)
                else:
                    # Remove the overlapping portion of the original signal
                    signal["stop_time"] = new_start_time
                    signal["data"] = signal["data"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    signal["high_freq"] = signal["high_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    signal["low_freq"] = signal["low_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    adjusted_signals.append(signal)

            elif signal["start_time"] < new_stop_time < signal["stop_time"]:
                # Case: The new signal overlaps the start of this signal
                signal["start_time"] = new_stop_time
                signal["data"] = signal["data"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                signal["high_freq"] = signal["high_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                signal["low_freq"] = signal["low_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                adjusted_signals.append(signal)

            elif new_start_time <= signal["start_time"] and new_stop_time >= signal["stop_time"]:
                # Case: The new signal completely overlaps this signal, so the original signal is removed

                continue
            else:
                # No overlap, keep the signal as is
                adjusted_signals.append(signal)

        # Add the new signal as well
        adjusted_signals.append({
            "type": new_signal_type,
            "data": new_signal_data["data"],
            "high_freq": new_signal_data["high_freq"],
            "low_freq": new_signal_data["low_freq"],
            "start_time": new_start_time,
            "stop_time": new_stop_time,
            "parameters": new_signal_parameters
        })

        self.signals = adjusted_signals
        self.plot_all_signals()  # Update the plot with the modified signals


    def adjust_previous_signals(self, new_start_time, new_stop_time):
        adjusted_signals = []
        for signal in self.signals:
            if signal["start_time"] < new_start_time < signal["stop_time"]:
                # Case: The new signal overlaps the end of this signal
                if new_stop_time < signal["stop_time"]:
                    # Trim the end of the original signal and keep the non-overlapping part
                    signal_part = {
                        "type": signal["type"],
                        "data": signal["data"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "high_freq": signal["high_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "low_freq": signal["low_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                        "start_time": signal["start_time"],
                        "stop_time": new_start_time,
                        "parameters": signal["parameters"]
                    }
                    adjusted_signals.append(signal_part)
                else:
                    # Remove the overlapping portion of the original signal
                    signal["stop_time"] = new_start_time
                    signal["data"] = signal["data"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    signal["high_freq"] = signal["high_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    signal["low_freq"] = signal["low_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)]
                    adjusted_signals.append(signal)
            elif signal["start_time"] < new_stop_time < signal["stop_time"]:
                # Case: The new signal overlaps the start of this signal
                signal["start_time"] = new_stop_time
                signal["data"] = signal["data"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                signal["high_freq"] = signal["high_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                signal["low_freq"] = signal["low_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):]
                adjusted_signals.append(signal)
            elif signal["start_time"] < new_start_time and signal["stop_time"] > new_stop_time:
                # Case: The new signal completely overlaps this signal
                signal_part1 = {
                    "type": signal["type"],
                    "data": signal["data"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                    "high_freq": signal["high_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                    "low_freq": signal["low_freq"][:int((new_start_time - signal["start_time"]) * TIME_STAMP)],
                    "start_time": signal["start_time"],
                    "stop_time": new_start_time,
                    "parameters": signal["parameters"]
                }
                signal_part2 = {
                    "type": signal["type"],
                    "data": signal["data"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):],
                    "high_freq": signal["high_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):],
                    "low_freq": signal["low_freq"][int((new_stop_time - signal["start_time"]) * TIME_STAMP):],
                    "start_time": new_stop_time,
                    "stop_time": signal["stop_time"],
                    "parameters": signal["parameters"]
                }
                adjusted_signals.extend([signal_part1, signal_part2])
            else:
                # No overlap, keep the signal as is
                adjusted_signals.append(signal)

        self.signals = adjusted_signals


    def set_custom_xlabel(self, xlabel, fontsize=9.5, color='black'):
        self.axes.set_xlabel('')  # Remove default xlabel
        self.axes.annotate(xlabel, xy=(1.01, -0.01), xycoords='axes fraction', fontsize=fontsize, color=color, ha='left', va='center')

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-qabstractitemmodeldatalist'):
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        # Get the dragged signal type
        item = event.source().selectedItems()[0]
        signal_type = item.text(0)

        # Determine if the signal is customized or imported
        if signal_type in self.app_reference.custom_signals or signal_type in self.app_reference.imported_signals:
            signal_data = self.get_signal_data(signal_type)
            if signal_data:
                start_time, stop_time = self.show_time_input_dialog(signal_type)
                if start_time is not None and stop_time is not None and stop_time > start_time:
                    # Perform segmentation to get high and low frequency components
                    high_freq_signal, low_freq_signal = self.segmentation_api.signal_segmentation(
                        product_signal=signal_data, sampling_rate=TIME_STAMP, downsample_rate=200
                    )

                    # Keep the original structure with "data" and add the new frequency components
                    signal_data = {
                        'data': signal_data,  # Original data is stored under "data" (unchanged)
                        'high_freq': high_freq_signal.tolist(),  # High frequency data
                        'low_freq': low_freq_signal.tolist()     # Low frequency data
                    }

                    # Print the lengths of data, high_freq, and low_freq
                    print(f"Original Data Length: {len(signal_data['data'])}, First 10 elements: {signal_data['data'][:10]}")
                    print(f"High Frequency Data Length: {len(signal_data['high_freq'])}, First 10 elements: {signal_data['high_freq'][:10]}")
                    print(f"Low Frequency Data Length: {len(signal_data['low_freq'])}, First 10 elements: {signal_data['low_freq'][:10]}")


                    # Check for overlapping signals and handle accordingly
                    if self.check_overlap(start_time, stop_time):
                        self.handle_overlap(start_time, stop_time, signal_type, signal_data, parameters=None)
                    else:
                        self.record_signal(signal_type, signal_data, start_time, stop_time, parameters=None)

        # If the signal is not in custom or imported signals, prompt for parameters
        else:
            parameters = self.prompt_signal_parameters(signal_type)
            if parameters is not None:
                start_time, stop_time = self.show_time_input_dialog(signal_type)
                if start_time is not None and stop_time is not None and stop_time > start_time:
                    signal_data = self.generate_signal_data(signal_type, parameters)

                    # Perform segmentation to get high and low frequency components
                    high_freq_signal, low_freq_signal = self.segmentation_api.signal_segmentation(
                        product_signal=signal_data, sampling_rate=TIME_STAMP, downsample_rate=200
                    )

                    # Keep the original structure with "data" and add the new frequency components
                    signal_data = {
                        'data': signal_data,  # Original data is stored under "data" (unchanged)
                        'high_freq': high_freq_signal.tolist(),  # High frequency data
                        'low_freq': low_freq_signal.tolist()     # Low frequency data
                    }

                    # Print the lengths of data, high_freq, and low_freq
                    print(f"Original Data Length: {len(signal_data['data'])}, First 10 elements: {signal_data['data'][:10]}")
                    print(f"High Frequency Data Length: {len(signal_data['high_freq'])}, First 10 elements: {signal_data['high_freq'][:10]}")
                    print(f"Low Frequency Data Length: {len(signal_data['low_freq'])}, First 10 elements: {signal_data['low_freq'][:10]}")


                    # Check for overlapping signals and handle accordingly
                    if self.check_overlap(start_time, stop_time):
                        self.handle_overlap(start_time, stop_time, signal_type, signal_data, parameters)
                    else:
                        self.record_signal(signal_type, signal_data, start_time, stop_time, parameters)

        # After recording the new signal, update the plot
        if self.signals:
            self.plot_all_signals()

        self.app_reference.actuator_signals[self.app_reference.current_actuator] = self.signals
        self.app_reference.update_actuator_text()
        self.app_reference.update_pushButton_5_state()


    def prompt_signal_parameters(self, signal_type):
        # Define mappings between signal types and dialogs
        signal_dialog_map = {
            "Sine": OscillatorDialog,
            "Square": OscillatorDialog,
            "Saw": OscillatorDialog,
            "Triangle": OscillatorDialog,
            "Chirp": ChirpDialog,
            "FM": FMDialog,
            "PWM": PWMDialog,
            "Noise": NoiseDialog,
        }
        
        # Get the dialog class based on signal type
        dialog_class = signal_dialog_map.get(signal_type)
        
        if dialog_class:
            # Try without parent if dialog is black in certain contexts
            dialog = dialog_class(signal_type)  # Removed 'self' as parent
            # Optional: Force update
            dialog.update()
            
            # Repaint the parent widget, if needed
            self.repaint()
            
            if dialog.exec() == QDialog.DialogCode.Accepted:
                config = dialog.get_config()
                # Add validation or logging here if needed
                if config:
                    return config
                else:
                    print(f"Invalid parameters for {signal_type}.")
        else:
            print(f"Unrecognized signal type: {signal_type}")
            
        return None  # Return None if dialog was canceled or invalid


    def record_signal(self, signal_type, signal_data, start_time, stop_time, parameters):
        """Record the signal to the timelinecanvas. In there the signal_data is unpacked to "data", "high_freq", and "low_freq" """
        # Record the signal data, including original, high frequency, and low frequency components
        print("Recorded")
        self.signals.append({
            "type": signal_type,
            "data": signal_data['data'],          # Store the original data as "data"
            "high_freq": signal_data['high_freq'],  # Store high frequency data
            "low_freq": signal_data['low_freq'],    # Store low frequency data
            "start_time": start_time,
            "stop_time": stop_time,
            "parameters": parameters
        })

    def plot_all_signals(self):
        # Set a variable to control which signal component to plot
        component_to_plot = 'data'  # Options: 'data', 'high_freq', 'low_freq'

        if not self.signals:
            # If no signals recorded, render a default plot with 10 seconds of 0 amplitude
            default_duration = 10  # seconds
            t = np.linspace(0, default_duration, TIME_STAMP * default_duration)
            signal_data = np.zeros_like(t)
            self.plot_signal_data(t, signal_data)
            return

        # Determine the max stop time across all recorded signals
        max_stop_time = max([signal["stop_time"] for signal in self.signals])

        # Store the signal duration for use in dragging functionality
        self.signal_duration = max_stop_time

        # Initialize an empty array of zeros for the full duration
        total_samples = int(max_stop_time * TIME_STAMP)
        combined_signal = np.zeros(total_samples)

        # Fill in the combined signal with each recorded signal's selected data component
        for signal in self.signals:
            start_sample = int(signal["start_time"] * TIME_STAMP)
            stop_sample = int(signal["stop_time"] * TIME_STAMP)
            signal_duration = stop_sample - start_sample

            # Use only the selected component (data, high_freq, low_freq)
            if component_to_plot in signal and len(signal[component_to_plot]) > 0:
                # Adjust the signal data to fit the required duration (stretch or truncate as needed)
                signal_data = np.tile(signal[component_to_plot], int(np.ceil(signal_duration / len(signal[component_to_plot]))))[:signal_duration]

                # If the adjusted signal_data is still too short, pad it with zeros
                if len(signal_data) < signal_duration:
                    signal_data = np.pad(signal_data, (0, signal_duration - len(signal_data)), 'constant')
            else:
                # Handle the case where the selected component is empty or not available
                signal_data = np.zeros(signal_duration)  # Fallback to an empty signal for this duration

            # Perform the assignment to the combined signal
            combined_signal[start_sample:stop_sample] = signal_data

        # Generate time array for the x-axis
        t = np.linspace(0, max_stop_time, total_samples)
        self.plot_signal_data(t, combined_signal)


    def plot_signal_data(self, t, signal_data):
        # Clear the current plot and plot the new signal
        self.axes.clear()
        
        # Set spine color and customize appearance
        spine_color = to_rgba((240/255, 235/255, 229/255))
        self.axes.spines['bottom'].set_color(spine_color)
        self.axes.spines['top'].set_color(spine_color)
        self.axes.spines['right'].set_color(spine_color)
        self.axes.spines['left'].set_color(spine_color)
        self.axes.tick_params(axis='x', colors=spine_color, labelsize=8)
        self.axes.tick_params(axis='y', colors=spine_color, labelsize=8)
        self.axes.set_ylabel('Amplitude', fontsize=9.5, color=spine_color)
        self.set_custom_xlabel('Time (s)', fontsize=9.5, color=spine_color)

        # Plot the signal data
        self.axes.plot(t, signal_data, color=spine_color)

        #draggg
        # Check if the signal is longer than 10 seconds
        if self.signal_duration > 2:
            self.axes.set_xlim(0, 2)  # Show only the first 10 seconds initially
            
            # Adjust arrow size and location using mutation_scale
            arrow_props = dict(facecolor='gray', edgecolor='none', alpha=0.6, mutation_scale=50)

            # Left arrow at the start of the plot
            self.axes.annotate('', xy=(0.035, 0.5), xytext=(-0.1, 0.5),
                            xycoords='axes fraction', textcoords='axes fraction',
                            arrowprops=dict(arrowstyle='-|>', **arrow_props))

            # Right arrow at the end of the plot
            self.axes.annotate('', xy=(0.965, 0.5), xytext=(1.1, 0.5),
                            xycoords='axes fraction', textcoords='axes fraction',
                            arrowprops=dict(arrowstyle='-|>', **arrow_props))

        # Draw the updated plot
        self.draw()


    def generate_signal_data(self, signal_type, parameters):
        # Generate the signal data based on the type and modified parameters
        t = np.linspace(0, parameters["duration"], int(TIME_STAMP * parameters["duration"]))
        if signal_type == "Sine":
            return np.sin(2 * np.pi * parameters["frequency"] * t).tolist()
        elif signal_type == "Square":
            return np.sign(np.sin(2 * np.pi * parameters["frequency"] * t)).tolist()
        elif signal_type == "Saw":
            return (2 * (t * parameters["frequency"] - np.floor(t * parameters["frequency"] + 0.5))).tolist()
        elif signal_type == "Triangle":
            return (2 * np.abs(2 * (t * parameters["frequency"] - np.floor(t * parameters["frequency"] + 0.5))) - 1).tolist()
        elif signal_type == "Chirp":
            instantaneous_frequency = parameters["frequency"] + parameters["rate"] * t
            if parameters["chirp_type"] == 'Sine':
                # Generate the sine waveform with time-varying frequency
                return np.sin(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
            elif parameters["chirp_type"] == 'Square':
                # Generate the square waveform with time-varying frequency
                return signal.square(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
            elif parameters["chirp_type"] == 'Saw':
                # Generate the sawtooth waveform with time-varying frequency
                return signal.sawtooth(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)
            elif parameters["chirp_type"] == 'Triangle':
                # Generate the triangle waveform with time-varying frequency
                return signal.sawtooth(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP, 0.5)
            else:
                return np.zeros_like(t).tolist()  # Default for unsupported types
        elif signal_type == "PWM":
            period = 1 / parameters["frequency"]
            return ((t % period) < (parameters["duty_cycle"] / 100) * period).astype(float)
        elif signal_type == "FM":
                    
            # Instantaneous phase
            instantaneous_phase = 2 * np.pi * parameters["frequency"] * t + parameters["index"] * np.sin(2 * np.pi * parameters["modulation"] * t)
            
            if parameters["FM_type"] == 'Sine':
                # Generate the sine FM signal
                return np.sin(instantaneous_phase)
            elif parameters["FM_type"] == 'Square':
                # Generate the square FM signal
                return signal.square(instantaneous_phase)
            elif parameters["FM_type"] == 'Saw':
                # Generate the sawtooth FM signal
                return signal.sawtooth(instantaneous_phase)
            elif parameters["FM_type"] == 'Triangle':
                # Generate the triangle FM signal
                return signal.sawtooth(instantaneous_phase, 0.5)
            else:
                return np.zeros_like(t).tolist()  # Default for unsupported types
        elif signal_type == "Noise":
            return (parameters["gain"] * np.random.normal(0, 1, len(t))).tolist()
        else: 
            return np.zeros_like(t).tolist()

    def get_signal_data(self, signal_type):
        # Retrieve signal data based on signal_type
        if signal_type in self.app_reference.custom_signals:
            return self.app_reference.custom_signals[signal_type]["data"]
        elif signal_type in self.app_reference.imported_signals:
            return self.app_reference.imported_signals[signal_type]["data"]
        return None

    def show_time_input_dialog(self, signal_type):
        dialog = TimeInputDialog(signal_type)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            start_time = dialog.start_time_input.value()
            stop_time = dialog.stop_time_input.value()
            return start_time, stop_time
        return None, None

class TimeInputDialog(QDialog):
    def __init__(self, signal_type, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Input Time Range")

        layout = QVBoxLayout(self)
        
        signal_label = QLabel(f"Signal Type: {signal_type}")
        
        layout.addWidget(signal_label)
        
        form_layout = QFormLayout()
        
        self.start_time_input = QDoubleSpinBox()
        self.start_time_input.setRange(0, 1000)  # Adjust range as needed
        form_layout.addRow("Start Time (s):", self.start_time_input)
        
        self.stop_time_input = QDoubleSpinBox()
        self.stop_time_input.setRange(0, 1000)  # Adjust range as needed
        form_layout.addRow("Stop Time (s):", self.stop_time_input)
        
        layout.addLayout(form_layout)
        
        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

class CanvasSizeDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Adjust Canvas Size")
        self.layout = QVBoxLayout(self)

        form_layout = QFormLayout()
        self.width_input = QLineEdit()
        self.height_input = QLineEdit()
        form_layout.addRow("Width (mm):", self.width_input)
        form_layout.addRow("Height (mm):", self.height_input)

        self.layout.addLayout(form_layout)

        button = QPushButton("OK")
        button.clicked.connect(self.accept)
        self.layout.addWidget(button)
        
class FloatingVerticalSlider(QSlider):
    def __init__(self, parent=None, app_reference=None):
        super().__init__(Qt.Orientation.Vertical, parent)
        self.app_reference = app_reference  # Store the reference to the main app
        self.setFixedWidth(10)
        # Apply the custom stylesheet to hide the handle
        self.setStyleSheet("""
            QSlider::groove:vertical {
                background: gray;
                width: 10px;
            }
            QSlider::handle:vertical {
                background: transparent;
                border: none;
                width: 0px;
                height: 0px;
            }
            QSlider::sub-page:vertical {
                background: transparent;
            }
        """)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # Set focus to ensure it responds to clicks
        self.slider_start_pos = None
        self.slider_movable = True  # Add a flag to control movement

        # Calculate the minimum and maximum x positions (3 cm from the left edge and 1 cm from the right edge)
        dpi = self.logicalDpiX()  # Get the screen DPI
        self.left_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # 3 cm in pixels
        self.right_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # 1 cm in pixels
        self.global_total_time = None

        # Store the initial vertical position to lock it
        self.initial_y = self.y()
        # Set initial position or size for the slider
        self.slider_position_ratio = 1


    def resizeEvent(self, event):
        """Override the resize event to adjust the slider's position when the window is resized."""
        # Calculate the ratio of the slider's current x position to the parent's width
        if self.parent().width() - self.left_offset - self.right_offset > 0:
            self.slider_position_ratio = (self.x() - self.left_offset) / (self.parent().width() - self.left_offset - self.right_offset)

        # Call the parent resizeEvent
        super(FloatingVerticalSlider, self).resizeEvent(event)

        # Now, update the movable range and slider's new position
        self.update_movable_range()

    def update_movable_range(self):
        """Recalculate the slider's movable range and adjust its position based on the window size."""
        dpi = self.logicalDpiX()
        self.left_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # 3 cm in pixels
        self.right_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # 1 cm in pixels

        # Ensure slider is within new bounds after resizing
        max_x = int(self.parent().width() - self.width() - self.right_offset)
        min_x = int(self.left_offset)

        # Calculate new slider position based on its previous ratio
        if hasattr(self, 'slider_position_ratio'):
            new_x = min_x + int(self.slider_position_ratio * (self.parent().width() - self.left_offset - self.right_offset))
        else:
            new_x = min_x  # Default to the minimum position if no previous ratio

        # Ensure the new_x stays within the movable range
        new_x = max(min_x, min(new_x, max_x))

        # Reposition the slider
        self.move(new_x, self.y())

    def set_slider_movable(self, movable):
        """Set whether the slider is movable horizontally."""
        self.slider_movable = movable

    def update_slider_height(self, height):
        self.setFixedHeight(height)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.slider_start_pos = event.globalPosition().toPoint()
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if not self.slider_movable:
            event.ignore()
            return

        if event.buttons() & Qt.MouseButton.LeftButton and self.slider_start_pos is not None:
            delta_x = event.globalPosition().x() - self.slider_start_pos.x()
            self.slider_start_pos = event.globalPosition().toPoint()

            new_x = int(self.x() + delta_x)  # Cast to int to ensure it's an integer
            max_x = int(self.parent().width() - self.width() - self.right_offset)  # Cast to int
            min_x = int(self.left_offset)  # Cast to int

            new_x = max(min_x, min(new_x, max_x))  # Ensure the slider is between 3 cm and 1 cm from edges

            self.move(int(new_x), self.y())  # Cast new_x to int before passing to move()

            # Update the current time position based on the slider's new location
            total_time = self.app_reference.calculate_total_time()
            if total_time > 0:
                time_position = total_time * (new_x - self.left_offset) / (self.parent().width() - self.left_offset - self.right_offset)
                self.app_reference.set_current_time_position_manually(time_position)

class Haptics_App(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # Get the absolute path to the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))

        # Construct the absolute path to the layout.ui file
        ui_file_path = os.path.join(current_dir, 'layout.ui')

        # Load the UI file
        self.ui = uic.loadUi(ui_file_path, self)
 
        self.resize(1500, 750)
        self.setWindowTitle("VibraForge GUI Editor")
        icon = QtGui.QIcon()
        icon_path = "resources/logo.jpg"

        self.statusBar().showMessage("Welcome to VibraForge GUI Editor")

        self.total_time = None

        if os.path.exists(icon_path):
            icon.addPixmap(QtGui.QPixmap(icon_path), QIcon.Mode.Normal, QIcon.State.Off)
            self.setWindowIcon(icon)
        else:
            print(f"Icon file not found at path: {icon_path}")

        self.threadpool = QtCore.QThreadPool()

        # Set main background color
        self.setStyleSheet("background-color: rgb(193, 205, 215);")
        self.widget_2.setStyleSheet("background-color: rgb(193, 205, 215);")
        self.widget_3.setStyleSheet("background-color: rgb(134, 150, 167);")


        # Initialize dictionaries to store signals
        self.custom_signals = {}  # Dictionary to store custom signals
        self.signal_templates = {}  # Dictionary to store provided signal templates
        self.imported_signals = {}  # Dictionary to store imported signals

        # Initialize the tree widget
        self.setup_tree_widget()

        # Add matplotlib canvas to the layout
        self.maincanvas = MplCanvas(self.ui.widget, width=5, height=1, dpi=100, app_reference=self)
        self.ui.gridLayout.addWidget(self.maincanvas, 0, 0, 1, 1)

        # Add ActuatorCanvas to the layout with a fixed height
        self.actuator_canvas = ActuatorCanvas(self.ui.widget_2,app_reference=self)
        self.actuator_canvas.setFixedHeight(380)  # Set the fixed height here
        self.ui.gridLayout_5.addWidget(self.actuator_canvas, 0, 0, 1, 1)

        # Create a scene for the selection bar
        self.selection_scene = QGraphicsScene()

        # Create the selection bar view and add it to the layout
        self.selection_bar = SelectionBar(self.selection_scene)
        self.selection_view = SelectionBarView(self.selection_scene, self.ui.widget_2)
        self.selection_view.setFixedSize(100, 100)  # Set size and position as needed
        self.ui.gridLayout_5.addWidget(self.selection_view, 0, 0, 1, 1)  # Overlay on the actuator canvas

        # Enable scroll bars for the timeline canvas
        self.ui.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.ui.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        # Connect clear button to clear_plot method
        self.ui.pushButton.clicked.connect(self.maincanvas.clear_plot)
        
        # Connect save button to save_current_signal method
        self.ui.pushButton_2.clicked.connect(self.save_current_signal)

        # Connect save button to save_current_signal method
        self.ui.pushButton_3.clicked.connect(self.clear_canvas_and_timeline)

        # Connect "Adjust Size" button to adjust_canvas_size method
        self.pushButton_4.clicked.connect(self.adjust_canvas_size)

        self.signal_counter = 1  # Counter for naming saved signals
        self.actionCreate_New_Chain.triggered.connect(self.create_actuator_branch)

        # Connect the import waveform action to the import_waveform method
        self.ui.actionImport_Waveform.triggered.connect(self.import_waveform)

        # Connect the actuator_added signal to the add_actuator_to_timeline slot
        self.actuator_canvas.actuator_added.connect(self.add_actuator_to_timeline)

        # Setup scroll area for timeline
        self.timeline_layout = QVBoxLayout(self.ui.scrollAreaWidgetContents)

        # Dictionary to store references to timeline widgets
        self.timeline_widgets = {}

        # Connect the properties_changed signal to the update_timeline_actuator slot
        self.actuator_canvas.properties_changed.connect(self.update_timeline_actuator)
        self.actuator_canvas.actuator_deleted.connect(self.remove_actuator_from_timeline)

        # Create the PreviewCanvas and add it to widget_3
        self.preview_canvas = PreviewCanvas(self.ui.widget_3, width=5, height=0.4, dpi=100)
        self.preview_canvas.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)

        # Create a layout for widget_3 if not already present
        layout = QtWidgets.QVBoxLayout(self.ui.widget_3)
        layout.setContentsMargins(0, 0, 0, 0)  # Remove any margins
        layout.setSpacing(0)  # Remove spacing between the canvas and the widget edges

        # Add the PreviewCanvas to widget_3's layout
        layout.addWidget(self.preview_canvas)

        # Create a layout for widget_3 if not already present
        self.ui.widget_3.setLayout(QtWidgets.QVBoxLayout())
        
        # Add the PreviewCanvas to widget_3's layout
        self.ui.widget_3.layout().addWidget(self.preview_canvas)

        # Ensure the tree widget itemClicked event is connected to update the preview
        self.ui.treeWidget.itemClicked.connect(self.on_tree_item_clicked)

        self.current_actuator = None  # Attribute to track the current actuator

        self.actuator_canvas.actuator_added.connect(self.connect_actuator_signals)

        # Connect the no_actuator_selected signal to the switch_to_main_canvas slot
        self.actuator_canvas.no_actuator_selected.connect(self.switch_to_main_canvas)

        # Add a dictionary to store signals for each actuator
        self.actuator_signals = {}

        # Initialize timeline_canvases as an empty dictionary
        self.timeline_canvases = {}

        # Instantiate DesignSaver
        self.design_saver = DesignSaver(self.actuator_canvas, self.timeline_canvases, self.maincanvas, self)

        # Connect the "Save As..." action to the save_design method
        self.ui.actionSave_New_Design.triggered.connect(self.design_saver.save_design)

        self.ui.actionStart_New_Design.triggered.connect(self.design_saver.load_design)


        # Load the Run and Pause icons

        # Initialize the slider_moving attribute
        self.slider_moving = False

        self.run_icon = self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay)
        self.pause_icon = self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPause)
        self.pushButton_5.setText("")
        
        # Set the initial icon to Run
        self.pushButton_5.setIcon(self.run_icon)
        
        # Connect the pushButton_5 click to the toggle_slider_movement method
        self.pushButton_5.clicked.connect(self.toggle_slider_movement)

        # setup the timeline timer
        self.timeline_thread = QThread()
        self.timeline_timer = TimelineTimer()
        self.timeline_timer.time_updated.connect(self.move_slider)
        self.timeline_timer.moveToThread(self.timeline_thread)
        self.timeline_thread.start()
        

        # Initialize the current time position
        self.current_time_position = 0  

        # Variables to control the movement
        self.slider_step = 2  # Adjust this value to control the speed of movement
        self.slider_target_pos = 0

        self.setup_slider_layer()

        # Call this method initially to set the state of pushButton_5
        self.update_pushButton_5_state()

        self.current_amplitudes = {}

        self.ble_api = python_ble_api()
        self.haptic_manager = HapticCommandManager(self.ble_api)

        self.ui.actionConnect_Bluetooth_Device.triggered.connect(self.show_bluetooth_connect_dialog)
        self.ui.actionDisconnect_Bluetooth_Device.triggered.connect(self.show_bluetooth_disconnect_dialog)

        self.bluetooth_connected = False
        self.ui.label.setText('<html>Bluetooth Status:</b> <span style="color:red;"><b>Not Connected</b></span></html>')
        
        
        self.ui.label.setStyleSheet("background-color: rgb(184, 199, 209);")
        self.ui.label_2.setStyleSheet("background-color: rgb(184, 199, 209);")
        self.ui.label_3.setStyleSheet("background-color: rgb(184, 199, 209);")
        self.ui.label_4.setStyleSheet("background-color: rgb(184, 199, 209);")
        

    def update_time_label(self, current_time_position):
        """Update the label with the current time."""
        _translate = QtCore.QCoreApplication.translate
        formatted_time = f"{abs(current_time_position):.3f} (s)"
        self.label_2.setText(_translate("MainWindow", f"Current Time: {formatted_time}"))


    def show_bluetooth_connect_dialog(self):
        """Show the Bluetooth connection dialog if no device is currently connected."""
        if not self.bluetooth_connected:
            dialog = BluetoothConnectDialog(self.ble_api, self)
            dialog.device_selected_signal.connect(self.update_bluetooth_connection_status)  # Handle connection signal
            dialog.exec()  # Show the dialog
        else:
            # If a device is already connected, show a warning message
            QtWidgets.QMessageBox.warning(self, "Bluetooth Connection",
                                        "A Bluetooth device is already connected. Please disconnect the current device first.")

    def show_bluetooth_disconnect_dialog(self):
        """Show the Bluetooth disconnection confirmation dialog."""
        if self.bluetooth_connected:
            dialog = BluetoothConnectDialog(self.ble_api, self)
            dialog.device_selected_signal.connect(self.update_bluetooth_disconnection_status)  # Handle disconnection signal
            dialog.show_disconnect_confirmation()  # Show the confirmation dialog
        else:
            # If no device is connected, show a message
            QtWidgets.QMessageBox.information(self, "No Device Connected", 
                                            "There is no Bluetooth device to disconnect.")

    def update_bluetooth_connection_status(self, success):
        """Update the connection status variable based on the connection result."""
        self.is_bluetooth_connected = success  # Update the connection status
        if success:
            self.bluetooth_connected = True
            self.ui.label.setText('<html>Bluetooth Status:</b> <span style="color:green;"><b>Connected</b></span></html>')
            print("Connected from Haptics")
        else:
            self.bluetooth_connected = False
            self.ui.label.setText('<html>Bluetooth Status:</b> <span style="color:red;"><b>Not Connected</b></span></html>')
            print("Failed from Haptics")

    def update_bluetooth_disconnection_status(self, success):
        """Update the connection status variable based on the disconnection result."""
        if success:
            print("Entered Here")
            self.bluetooth_connected = False  # Update the connection status
            self.ui.label.setText('<html>Bluetooth Status:</b> <span style="color:red;"><b>Not Connected</b></span></html>')
            self.statusBar().showMessage("Bluetooth device disconnected successfully.")
        else:
            self.statusBar().showMessage("Bluetooth disconnection failed.")

    def update_pushButton_5_state(self):
        """Update the state of pushButton_5 based on whether any actuators have signals."""
        # Check if any actuator has signals
        has_signals = any(self.actuator_signals.values())

        if has_signals:
            self.pushButton_5.setEnabled(True)
            self.update_slider_target_position()
            self.floating_slider.set_slider_movable(True)  
        else:
            self.pushButton_5.setEnabled(False)
            self.slider_target_pos = 0  # No movement target
            # Set the initial position 3 cm away from the left edge
            dpi = self.logicalDpiX()  # Get the screen DPI
            cm_to_pixels = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 3 cm to pixels
            initial_position = int(cm_to_pixels)  # 3 cm in pixels
            self.floating_slider.move(initial_position, self.floating_slider.y())  # Set the initial position
            self.floating_slider.set_slider_movable(False)  # Disable slider movement
    

    def update_slider_target_position(self):
        """Update the target position for the slider based on the maximum stop time."""
        max_stop_time = self.calculate_total_time()

        # Calculate the target position in pixels based on the max stop time and total time
        if max_stop_time and self.total_time:
            dpi = self.logicalDpiX()
            cm_to_pixels = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 3 cm to pixels
            adjusted_width = self.ui.scrollAreaWidgetContents.width() - 2 * cm_to_pixels

            # Calculate the target position based on the time ratio
            self.slider_target_pos = int(adjusted_width * (max_stop_time / self.total_time) + cm_to_pixels)

    def toggle_slider_movement(self):
        """Toggle slider movement and switch button icons."""
        if self.slider_moving:
            self.pause_slider_movement()
            self.haptic_manager.stop_playback()
        else:
            self.start_slider_movement()
            self.haptic_manager.start_playback()

    def start_slider_movement(self):
        """Start moving the slider based on the current slider position."""
        self.start_time = time.time() - self.current_time_position  # Adjust start time based on current slider position
        self.slider_moving = True
        self.timeline_timer.play()  # Timer interval for updating the slider position
        self.pushButton_5.setIcon(self.pause_icon)
        self.actuator_canvas.setEnabled(False)

    def pause_slider_movement(self):
        """Pause the slider movement."""
        self.timeline_timer.pause()
        self.slider_moving = False
        self.pushButton_5.setIcon(self.run_icon)  # Switch back to Run icon
        self.actuator_canvas.setEnabled(True)

    # calculate the longest play time of all signals
    def calculate_total_time(self):
        all_stop_times = []
        for signals in self.actuator_signals.values():
            all_stop_times.extend([signal["stop_time"] for signal in signals])
        return max(all_stop_times) if all_stop_times else 0

    def move_slider(self, timeline_time):
        """Move the slider in real time based on the signal's total time."""
        if self.slider_moving:
            self.current_time_position = timeline_time
            # Calculate the total time of all signals
            total_time = self.calculate_total_time()

            # Ensure total_time is valid
            if total_time > 0:
                # Update the label with the current time
                self.update_time_label(self.current_time_position)

                # Calculate the time ratio
                time_ratio = self.current_time_position / total_time

                # Calculate the new slider position based on the time ratio
                dpi = self.logicalDpiX()
                cm_to_pixels = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 3 cm to pixels
                adjusted_width = self.ui.scrollAreaWidgetContents.width() - 2 * cm_to_pixels
                new_pos = int(time_ratio * adjusted_width + cm_to_pixels)

                # Move the slider to the new position
                self.floating_slider.move(int(new_pos), self.floating_slider.y())

                # Highlight actuators and update signals based on the new time position
                current_amplitudes = self.update_current_amplitudes(self.current_time_position)
                self.actuator_canvas.highlight_actuators_at_time(self.current_time_position)
            else:
                print("Warning: No signals found or invalid total time.")
                self.timeline_timer.reset()
                self.slider_moving = False
                self.pushButton_5.setIcon(self.run_icon)
                self.actuator_canvas.setEnabled(True)
                return

            # Stop the slider when it reaches the end of the total time
            if self.current_time_position >= total_time:
                self.timeline_timer.reset()
                self.slider_moving = False
                self.pushButton_5.setIcon(self.run_icon)
                self.actuator_canvas.setEnabled(True)
                self.haptic_manager.stop_playback()

                # Reset to the beginning when reaching the end
                self.current_time_position = 0  # Reset current position to 0 for next play

            # Update the haptic manager with the new signal information
            # print(f"timeline time = {self.current_time_position}, time = {time.perf_counter()}")
            self.haptic_manager.update(current_amplitudes)

    def set_current_time_position_manually(self, time_position):
        """Set the current time position manually and update the slider position."""
        self.current_time_position = time_position
        self.timeline_timer.manual_update(self.current_time_position)
        self.update_time_label(self.current_time_position)
        self.update_current_amplitudes(self.current_time_position)
        self.actuator_canvas.highlight_actuators_at_time(self.current_time_position)


    def setup_slider_layer(self):
        # Create a QWidget that acts as a layer for the slider
        self.slider_layer = QWidget(self.ui.scrollAreaWidgetContents)
        self.slider_layer.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.slider_layer.setGeometry(self.ui.scrollAreaWidgetContents.rect())
        self.slider_layer.setStyleSheet("background: transparent;")
        
        # Add a vertical slider to float over the timeline layout
        self.floating_slider = FloatingVerticalSlider(self.slider_layer, app_reference=self)
        self.floating_slider.setFixedHeight(self.ui.scrollAreaWidgetContents.height())
        self.floating_slider.update_movable_range()

        # Set the initial position 3 cm away from the left edge
        dpi = self.logicalDpiX()  # Get the screen DPI
        cm_to_pixels = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 3 cm to pixels
        initial_position = int(cm_to_pixels)  # 3 cm in pixels
        self.floating_slider.move(initial_position, self.floating_slider.y())  # Set the initial position

        # Ensure the slider layer and slider are on top
        self.raise_slider_layer()

        # Install an event filter to track resizing and adjust the slider
        self.ui.scrollAreaWidgetContents.installEventFilter(self)

    def raise_slider_layer(self):
        # Raise the slider layer and slider to ensure they are on top
        self.slider_layer.raise_()
        self.floating_slider.raise_()

    def eventFilter(self, source, event):
        if event.type() == QEvent.Type.Resize and source is self.ui.scrollAreaWidgetContents:
            # Adjust the slider layer to match the size of the scroll area content
            self.slider_layer.setGeometry(self.ui.scrollAreaWidgetContents.rect())
            self.floating_slider.update_slider_height(self.ui.scrollAreaWidgetContents.height())
            self.raise_slider_layer()  # Ensure the slider layer stays on top after resizing
        return super(Haptics_App, self).eventFilter(source, event)



    def update_current_amplitudes(self, time_position):
        # Tracing the low frequency data

        self.current_amplitudes.clear()

        for actuator_id, signals in self.actuator_signals.items():
            for signal in signals:
                if signal["start_time"] <= time_position <= signal["stop_time"]:
                    signal_duration = signal["stop_time"] - signal["start_time"]
                    relative_position = (time_position - signal["start_time"]) / signal_duration
                    index = int(relative_position * len(signal["low_freq"]))
                    
                    # Ensure index is within bounds
                    index = max(0, min(index, len(signal["low_freq"]) - 1))
                    
                    amplitude = signal["low_freq"][index]
                    frequency = signal["high_freq"][index]
                    
                    # Update current_amplitudes with only the amplitude and frequency
                    self.current_amplitudes[actuator_id] = {
                        "current_amplitude": amplitude,
                        "current_frequency": frequency,
                    }

                    break  # Only handle one signal per actuator at a time

        # Return both the current_amplitudes
        return self.current_amplitudes

    def update_actuator_text(self):
        # Find the global largest stop time across all actuators
        all_stop_times = []
        for signals in self.actuator_signals.values():
            all_stop_times.extend([signal["stop_time"] for signal in signals])

        if all_stop_times:
            global_total_time = max(all_stop_times)
        else:
            global_total_time = 1  # Avoid division by zero in the width calculation

        # Define the 3 cm left offset and 1 cm right offset in pixels
        dpi = self.logicalDpiX()  # Get the screen DPI
        left_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 3 cm to pixels
        right_offset = (OS_DEPENDENT_VALUE / 2.54) * dpi  # Convert 1 cm to pixels

        # Update the visual timeline for each actuator widget
        for actuator_id, (actuator_widget, actuator_label) in self.timeline_widgets.items():
            if actuator_id in self.actuator_signals:
                signals = self.actuator_signals[actuator_id]

                # Remove all existing signal widgets from the actuator widget layout, but keep the ID and type
                for i in reversed(range(1, actuator_widget.layout().count())):
                    item = actuator_widget.layout().takeAt(i)
                    widget = item.widget()
                    if widget:
                        widget.deleteLater()
                    else:
                        del item  # Remove spacers

                # Set up the layout for the actuator widget if not already done
                if not actuator_widget.layout():
                    layout = QtWidgets.QHBoxLayout()
                    actuator_widget.setLayout(layout)
                    layout.setContentsMargins(0, 0, 0, 0)
                    layout.setSpacing(5)  # Add spacing between the ID/Type and the signals

                # Ensure the ID/Type label stays in the first position
                if actuator_label.parent() is None:
                    actuator_widget.layout().insertWidget(0, actuator_label)

                # Create a container for the timeline and signal widgets
                timeline_container = QtWidgets.QWidget(actuator_widget)
                timeline_container.setStyleSheet("background-color: transparent;")
                timeline_container.setFixedHeight(30)
                timeline_layout = QtWidgets.QHBoxLayout(timeline_container)
                timeline_layout.setContentsMargins(0, 0, 0, 0)
                timeline_layout.setSpacing(0)

                # Calculate the available width of the actuator widget after applying the offsets
                widget_width = actuator_widget.size().width() - left_offset - right_offset  # Subtract both offsets

                # Track the last stop time to insert gaps
                last_stop_time = 0

                # Sort signals by start time
                signals.sort(key=lambda signal: signal["start_time"])

                for signal in signals:
                    # Calculate the relative width of the signal widget based on its duration
                    signal_duration = signal["stop_time"] - signal["start_time"]
                    signal_width_ratio = signal_duration / global_total_time
                    signal_width = int(signal_width_ratio * widget_width)

                    # Calculate the relative starting position of the signal widget
                    signal_start_ratio = signal["start_time"] / global_total_time
                    signal_start_position = int(signal_start_ratio * widget_width) + left_offset  # Add the 3 cm offset

                    # If there is a gap between the last signal's stop time and this signal's start time, add a spacer
                    if signal["start_time"] > last_stop_time:
                        gap_duration = signal["start_time"] - last_stop_time
                        gap_width_ratio = gap_duration / global_total_time
                        gap_width = int(gap_width_ratio * widget_width)

                        # Add a spacer to represent the gap
                        spacer = QtWidgets.QSpacerItem(gap_width, 30, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
                        timeline_layout.addItem(spacer)

                    # Create the signal widget, making it smaller vertically and with rounded corners
                    signal_widget = QtWidgets.QLabel(f'{signal["type"]} ({", ".join([f"{k}: {v}" for k, v in (signal["parameters"] or {}).items()])})')
                    signal_widget.setFixedSize(signal_width, 30)  # Set smaller height for the signal widget
                    signal_widget.setStyleSheet("""
                        background-color: rgba(100, 150, 250, 150); 
                        color: white; 
                        border-radius: 7px;
                        padding: 3px;
                    """)

                    # Add the signal widget to the layout
                    timeline_layout.addWidget(signal_widget)
                    # Lower the signal widget to the bottom layer
                    signal_widget.lower()

                    # Update the last stop time
                    last_stop_time = signal["stop_time"]

                # Add a stretch to fill the remaining space
                timeline_layout.addStretch()

                # Add the timeline container to the actuator widget after the ID and type
                actuator_widget.layout().addWidget(timeline_container)
                timeline_container.updateGeometry()

        # After adding all timeline widgets, ensure the slider layer stays on top
        self.raise_slider_layer()

    def resizeEvent(self, event):
        """Override the resize event to update the timeline and slider when the window size changes."""
        super().resizeEvent(event)
        
        # Recalculate positions of timeline containers
        self.update_actuator_text()

        # Ensure the slider layer stays on top after resizing
        self.raise_slider_layer()

                  
    def connect_actuator_signals(self, actuator_id, actuator_type, color, x, y):
        actuator = self.actuator_canvas.get_actuator_by_id(actuator_id)
        if actuator:
            actuator.signal_handler.clicked.connect(self.on_actuator_clicked)
            actuator.signal_handler.properties_changed.connect(self.update_plotter)

    def update_plotter(self, actuator_id, actuator_type, color):
        if self.current_actuator == actuator_id:
            self.switch_to_timeline_canvas(actuator_id)  # Update the plotter to reflect changes

    def switch_to_timeline_canvas(self, actuator_id):
        # Clear the current layout
        self.ui.gridLayout.removeWidget(self.maincanvas)
        self.maincanvas.setParent(None)  # Detach MplCanvas from its parent

        # Create and add the TimelineCanvas
        color_rgb = self.actuator_canvas.branch_colors[actuator_id.split('.')[0]].getRgbF()[:3]
        self.timeline_canvas = TimelineCanvas(self.ui.widget, color=color_rgb, label=f"Timeline for {actuator_id}", app_reference=self)
        self.ui.gridLayout.addWidget(self.timeline_canvas, 0, 0, 1, 1)

        # Store the TimelineCanvas in the dictionary
        self.timeline_canvases[actuator_id] = self.timeline_canvas

        # Retrieve and plot the signal data for this actuator
        if actuator_id in self.actuator_signals:
            self.timeline_canvas.signals = self.actuator_signals[actuator_id]
            self.timeline_canvas.plot_all_signals()

    def switch_to_main_canvas(self):
        # Check if already on MplCanvas, no need to switch if it is
        if self.ui.gridLayout.indexOf(self.maincanvas) != -1:
            return

        # Remove the current widget (TimelineCanvas)
        if hasattr(self, 'timeline_canvas'):
            self.ui.gridLayout.removeWidget(self.timeline_canvas)
            self.timeline_canvas.setParent(None)

        # Add the MplCanvas back to the layout
        self.ui.gridLayout.addWidget(self.maincanvas, 0, 0, 1, 1)
        self.current_actuator = None  # Reset current actuator tracking

    def on_actuator_clicked(self, actuator_id):
        # When an actuator is clicked, switch to the TimelineCanvas
        if self.current_actuator != actuator_id:
            self.current_actuator = actuator_id
            self.switch_to_timeline_canvas(actuator_id)
            # Retrieve the actuator's type and color
            actuator = self.actuator_canvas.get_actuator_by_id(actuator_id)
            if actuator:
                # Immediately update the plotter to reflect the clicked actuator
                self.update_plotter(actuator_id, actuator.actuator_type, actuator.color.name())

    def mousePressEvent(self, event):
        # If clicked on blank space, switch back to MplCanvas
        if self.current_actuator and not self.actuator_canvas.itemAt(event.pos()):
            self.current_actuator = None
            self.switch_to_main_canvas()
        super().mousePressEvent(event)

    def update_status_bar(self, signal_type, parameters):
        # Format the parameters into a readable string
        param_str = ', '.join([f'{key}: {value}' for key, value in parameters.items()])
        # Display the signal type and parameters in the status bar
        self.statusBar().showMessage(f"Current Signal: {signal_type} | Parameters: {param_str}")

    def clear_canvas_and_timeline(self, bypass_dialog=False):
        if not bypass_dialog:
            # Prompt a warning to the user
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("Clear Actuator and Timeline Data")
            msg_box.setText("Are you sure you want to clear all the actuators and corresponding timeline data?")
            msg_box.setIcon(QMessageBox.Icon.Warning)
            msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            
            # Apply the custom stylesheet for the message box
            msg_box.setStyleSheet("""
                QMessageBox { background-color: white; }
                QLabel { color: black; }
                QPushButton { 
                    background-color: white; 
                    color: black; 
                    border: 1px solid black; 
                    padding: 5px; 
                }
                QPushButton:hover { 
                    background-color: gray; 
                }
            """)

            result = msg_box.exec()

            if result == QMessageBox.StandardButton.Yes:
                # If user confirms, clear the canvas and timeline
                self.actuator_canvas.clear_lines_except_scale()  # Clear lines, not the scale line
                self.actuator_canvas.clear_canvas()  # Clear actuators
                self.clear_timeline_canvas()  # Clear timeline
                self.reset_color_management()
                self.switch_to_main_canvas()
                self.update_pushButton_5_state()
                
            else:
                # If user cancels, do nothing
                return
        else:
            # Bypass dialog and clear the canvas and timeline directly
            self.actuator_canvas.clear_lines_except_scale()  # Clear lines, not the scale line
            self.actuator_canvas.clear_canvas()  # Clear actuators
            self.clear_timeline_canvas()  # Clear timeline
            self.reset_color_management()
            self.switch_to_main_canvas()

    def clear_timeline_canvas(self):
        # Clear the timeline layout
        while self.timeline_layout.count() > 0:
            item = self.timeline_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        self.timeline_widgets.clear()
        self.actuator_signals.clear()  # Clear the stored signals

    def reset_color_management(self):
        # Reset color management stuff
        self.actuator_canvas.branch_colors.clear()
        self.actuator_canvas.color_index = 0

    def adjust_canvas_size(self):
        dialog = CanvasSizeDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            try:
                width = int(dialog.width_input.text())
                height = int(dialog.height_input.text())
                self.actuator_canvas.set_canvas_size(width, height)
            except ValueError:
                print("Invalid input. Please enter valid integer values for width and height.")
    
    def add_actuator_to_timeline(self, new_id, actuator_type, color, x, y):
        # Convert the color to a suitable format for the stylesheet
        color_rgb = QColor(color).getRgbF()[:3]
        color_style = f"background-color: rgba({int(color_rgb[0]*255)}, {int(color_rgb[1]*255)}, {int(color_rgb[2]*255)}, 255);"

        # Create a placeholder widget instead of TimelineCanvas
        actuator_widget = QWidget(parent=self.ui.scrollAreaWidgetContents)
        actuator_widget.setStyleSheet(color_style)  # Apply the background color

        # Add a simple label to represent the actuator in the timeline area
        actuator_layout = QHBoxLayout(actuator_widget)
        actuator_label = QLabel(f"{actuator_type} - {new_id}")
        actuator_label.setStyleSheet("color: white;")  # Ensure text is visible
        actuator_layout.addWidget(actuator_label)
        
        # Add the widget to the timeline layout
        self.timeline_layout.addWidget(actuator_widget)
        self.timeline_widgets[new_id] = (actuator_widget, actuator_label)

        self.raise_slider_layer()

        self.update_pushButton_5_state()

    def update_timeline_actuator(self, old_actuator_id, new_actuator_id, actuator_type, color):
        if old_actuator_id in self.timeline_widgets:
            actuator_widget, actuator_label = self.timeline_widgets.pop(old_actuator_id)
            
            # Convert the color to a suitable format for the stylesheet
            color_rgb = QColor(color).getRgbF()[:3]
            color_style = f"background-color: rgba({int(color_rgb[0]*255)}, {int(color_rgb[1]*255)}, {int(color_rgb[2]*255)}, 255);"
            
            # Update the widget's background color
            actuator_widget.setStyleSheet(color_style)
            
            # Update the label text
            actuator_label.setText(f"{actuator_type} - {new_actuator_id}")
            
            # Store the updated reference with the new ID
            self.timeline_widgets[new_actuator_id] = (actuator_widget, actuator_label)
            
            # Update the actuator_signals dictionary to reflect the ID change
            if old_actuator_id in self.actuator_signals:
                self.actuator_signals[new_actuator_id] = self.actuator_signals.pop(old_actuator_id)

            # Immediately update the plotter to reflect the changes
            self.update_plotter(new_actuator_id, actuator_type, color)
            
    def remove_actuator_from_timeline(self, actuator_id):
        if actuator_id in self.timeline_widgets:
            actuator_widget, actuator_label = self.timeline_widgets.pop(actuator_id)
            self.timeline_layout.removeWidget(actuator_widget)
            actuator_widget.deleteLater()  # Properly delete the widget
            self.update_pushButton_5_state()

        # Remove the associated signal data
        if actuator_id in self.actuator_signals:
            del self.actuator_signals[actuator_id]

    def import_waveform(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Import Waveform", "", "CSV Files (*.csv);;All Files (*)")
        if file_path:
            try:
                # Ask the user to input the sampling rate
                sampling_rate, ok = QInputDialog.getInt(
                    self, 
                    "Sampling Rate Input", 
                    "Please enter the sampling rate (in Hz):", 
                    min=1, 
                    max=192000, 
                    value=44100  # Default value
                )

                if not ok:
                    return  # User canceled input, exit

                # Read the CSV file
                data = self.read_csv_file(file_path)
                if sampling_rate != TIME_STAMP:
                    current_signal_length = len(data)
                    resample_factor = sampling_rate / TIME_STAMP
                    data = np.interp(
                        np.linspace(0, current_signal_length, int(current_signal_length / resample_factor)),
                        np.arange(0, current_signal_length),
                        data
                    )
                    sampling_rate = TIME_STAMP
                print(f"CSV Data: {data}")  # Debugging print statement

                # Extract the signal type from the CSV filename
                signal_type = os.path.splitext(os.path.basename(file_path))[0]
                print(f"Signal Type: {signal_type}")  # Debugging print statement
                print("data length", len(data))
                # Convert CSV data to the required format with the max value as gain
                waveform_data = self.convert_csv_to_waveform_format(data, signal_type, sampling_rate)

                if waveform_data:
                    self.add_imported_waveform(file_path, waveform_data)

            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to import waveform: {e}")

    def read_csv_file(self, file_path):
        """
        Reads a CSV file and converts it to a list of rows.
        Each row is considered a data point corresponding to a timestamp.
        """
        with open(file_path, 'r') as csv_file:
            reader = csv.reader(csv_file)
            return [float(row[0]) for row in reader]  # Convert each row to float

    def convert_csv_to_waveform_format(self, csv_data, signal_type, sampling_rate):
        """
        Converts the CSV data into the specified JSON format for waveforms.
        The signal type is derived from the CSV filename.
        The gain is the maximum value out of all data numbers in the CSV.
        """
        try:
            # Flatten the CSV data to find the maximum value
            amplitude = max(csv_data)  # Set the gain as the maximum value in the CSV
            print(f"Max Amplitude (Gain): {amplitude}")  # Debugging print statement


            # Convert the CSV data to the format you described
            waveform_format = {
                "value0": {
                    "gain": amplitude,
                    "bias": 0.0,
                    "sampling_rate": sampling_rate,
                    "m_ptr": {
                        "polymorphic_id": 2147483649,
                        "polymorphic_name": f"tact::Signal::Model<tact::{signal_type}>",
                        "ptr_wrapper": {
                            "valid": 1,
                            "data": {
                                "Concept": {},
                                "m_model": {
                                    "IOscillator": {
                                        "x": {
                                            "gain": 1.0,
                                            "bias": 0.0,
                                            "m_ptr": {
                                                "polymorphic_id": 2147483650,
                                                "polymorphic_name": "tact::Signal::Model<tact::Time>",
                                                "ptr_wrapper": {
                                                    "valid": 1,
                                                    "data": {
                                                        "Concept": {},
                                                        "m_model": {}
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                },
                "data": csv_data  # CSV data as a list of rows
            }

            return waveform_format

        except Exception as e:
            print(f"Error while converting CSV to waveform format: {e}")
            return None

    def add_imported_waveform(self, file_path, waveform_data):
        imports_item = None
        # Find or create the Imports item in the tree widget
        for i in range(self.ui.treeWidget.topLevelItemCount()):
            top_item = self.ui.treeWidget.topLevelItem(i)
            if top_item.text(0) == "Imported Signals":
                imports_item = top_item
                break
        if imports_item is None:
            imports_item = QTreeWidgetItem(self.ui.treeWidget)
            imports_item.setText(0, "Imported Signals")
            imports_item.setToolTip(0, "Imported Signals")

        waveform_name = os.path.basename(file_path)
        child = QTreeWidgetItem(imports_item)
        child.setText(0, waveform_name)
        child.setToolTip(0, waveform_name)
        self.imported_signals[waveform_name] = waveform_data
        child.setFlags(child.flags() | QtCore.Qt.ItemFlag.ItemIsEditable)  # Make the item editable
        child.setData(0, QtCore.Qt.ItemDataRole.UserRole, waveform_name)  # Store the original name

    def create_actuator_branch(self):
        dialog = CreateBranchDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            num_actuators = dialog.num_actuators_input.value()
            lra_count = dialog.lra_input.value()
            vca_count = dialog.vca_input.value()
            m_count = dialog.m_input.value()
            grid_pattern = dialog.grid_pattern_input.text()

            self.actuator_canvas.create_actuator_branch(
                num_actuators, lra_count, vca_count, m_count, grid_pattern)

    def setup_tree_widget(self):
        tree = self.ui.treeWidget
        tree.setHeaderHidden(True)
        tree.setStyleSheet("""
            QTreeWidget {
                background-color: rgb(134, 150, 167);
                color: rgb(240, 235, 229);
            }
            QToolTip {
                color: rgb(134, 150, 167);  /* Text color */
                background-color: rgba(0, 0, 0, 128);  /* Black background with transparency */
                font-weight: bold;  /* Bold text */
            }
        """)
        tree.setDragEnabled(True)

        tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        tree.setDefaultDropAction(QtCore.Qt.DropAction.MoveAction)
        tree.setToolTipDuration(2000)  # Set tooltip duration to 2 seconds


        # Disable drag by default
        tree.setDragEnabled(False)

        # Connect to itemPressed to control dragging behavior
        tree.itemPressed.connect(self.on_tree_item_pressed)
        
        # Create top-level items
        oscillators = QTreeWidgetItem(tree)
        oscillators.setText(0, "Oscillators")
        oscillators.setToolTip(0, "Oscillators")  # Set tooltip


        self.customizes = QTreeWidgetItem(tree)
        self.customizes.setText(0, "Customized Signals")
        self.customizes.setToolTip(0, "Customized Signals")  # Set tooltip

        self.imports = QTreeWidgetItem(tree)
        self.imports.setText(0, "Imported Signals")
        self.imports.setToolTip(0, "Imported Signals")  # Set tooltip

        # Add child items to "Oscillators"
        osc_items = ["Sine", "Square", "Saw", "Triangle", "Chirp", "FM", "PWM", "Noise"]
        for item in osc_items:
            child = QTreeWidgetItem(oscillators)
            child.setText(0, item)
            child.setToolTip(0, item)  # Set tooltip
            self.signal_templates[item] = self.generate_signal(item)

        # Expand all items by default
        tree.expandAll()

        # Connect tree widget item selection to the plotting function
        tree.itemClicked.connect(self.on_tree_item_clicked)

        # Connect the itemChanged signal to handle renaming
        tree.itemChanged.connect(self.on_tree_item_changed)

        # Enable context menu
        tree.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        tree.customContextMenuRequested.connect(self.on_custom_context_menu)

    def on_tree_item_pressed(self, item, column):
        # Enable dragging only if the item is a child item (i.e., it has a parent)
        if item.parent() is not None:
            self.ui.treeWidget.setDragEnabled(True)
        else:
            self.ui.treeWidget.setDragEnabled(False)

    @pyqtSlot(QTreeWidgetItem, int)
    def on_tree_item_clicked(self, item, column):
        if item is None or not isinstance(item, QTreeWidgetItem):
            return  # Exit if the clicked item is None or not a valid item
        
        signal_type = item.text(column)
        if signal_type in self.custom_signals:  # Check if it's a custom signal
            custom_signal = self.custom_signals.get(signal_type)
            if custom_signal is not None:
                self.preview_canvas.plot_default_signal(custom_signal)
        elif signal_type in self.signal_templates:  # Check if it's a provided signal template
            template_signal = self.signal_templates.get(signal_type)
            if template_signal is not None:
                self.preview_canvas.plot_default_signal(template_signal)
        elif signal_type in self.imported_signals:  # Check if it's an imported signal
            imported_signal = self.imported_signals.get(signal_type)
            if imported_signal is not None:
                self.preview_canvas.plot_default_signal(imported_signal)
        else:
            self.preview_canvas.plot_default_signal(None)  # Clear the preview canvas if no valid signal is found

    def generate_signal(self, signal_type):
        base_signal = {
            "value0": {
                "gain": 1.0,
                "bias": 0.0,
                "m_ptr": {
                    "polymorphic_id": 2147483649,
                    "polymorphic_name": f"tact::Signal::Model<tact::{signal_type}>",
                    "ptr_wrapper": {
                        "valid": 1,
                        "data": {
                            "Concept": {},
                            "m_model": {
                                "IOscillator": {
                                    "x": {
                                        "gain": 628.3185307179587,
                                        "bias": 0.0,
                                        "m_ptr": {
                                            "polymorphic_id": 2147483650,
                                            "polymorphic_name": "tact::Signal::Model<tact::Time>",
                                            "ptr_wrapper": {
                                                "valid": 1,
                                                "data": {
                                                    "Concept": {},
                                                    "m_model": {}
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "data": []
        }

        t = np.linspace(0, 1, TIME_STAMP).tolist()  # Convert numpy array to list
        if signal_type == "Sine":
            base_signal["data"] = np.sin(2 * np.pi * 10 * np.array(t)).tolist()
        elif signal_type == "Square":
            base_signal["data"] = np.sign(np.sin(2 * np.pi * 10 * np.array(t))).tolist()
        elif signal_type == "Saw":
            base_signal["data"] = (2 * (np.array(t) - np.floor(np.array(t) + 0.5))).tolist()
        elif signal_type == "Triangle":
            base_signal["data"] = (2 * np.abs(2 * (np.array(t) - np.floor(np.array(t) + 0.5))) - 1).tolist()

        elif signal_type == "Chirp":
            instantaneous_frequency = 10 + 50 * np.array(t)
            # Generate the sine waveform with time-varying frequency
            base_signal["data"] = (np.sin(2 * np.pi * np.cumsum(instantaneous_frequency) / TIME_STAMP)).tolist()
        elif signal_type == "FM":
            instantaneous_phase = 2 * np.pi * 10 * np.array(t) + 2 * np.sin(2 * np.pi * 10 * np.array(t))
            base_signal["data"] = (np.sin(instantaneous_phase)).tolist()

        elif signal_type == "PWM":
            period = 1 / 10
            base_signal["data"]= (((np.array(t) % period) < (70 / 100) * period).astype(float)).tolist()
    
        elif signal_type == "Noise":
            base_signal["data"] = np.random.normal(0, 1, len(t)).tolist()

        else:
            base_signal["data"] = np.zeros_like(t).tolist()

        return base_signal

    def signal_exists(self, signal):
        for existing_signal in self.custom_signals.values():
            if np.array_equal(existing_signal, signal):
                return True
        for existing_signal in self.signal_templates.values():
            if np.array_equal(existing_signal, signal):
                return True
        return False

    def save_current_signal(self):
        print("start save")
        # Continue with saving for subsequent signals
        if self.maincanvas.current_signal is not None:
            signal_data = {
                "value0": {
                    "gain": 1.0,
                    "bias": 0.0,
                    "sampling_rate": self.maincanvas.current_signal_sampling_rate,
                    "m_ptr": {
                        "polymorphic_id": 2147483649,
                        "polymorphic_name": f"tact::Signal::Model<tact::Custom>",
                        "ptr_wrapper": {
                            "valid": 1,
                            "data": {
                                "Concept": {},
                                "m_model": {
                                    "IOscillator": {
                                        "x": {
                                            "gain": 628.3185307179587,
                                            "bias": 0.0,
                                            "m_ptr": {
                                                "polymorphic_id": 2147483650,
                                                "polymorphic_name": "tact::Signal::Model<tact::Time>",
                                                "ptr_wrapper": {
                                                    "valid": 1,
                                                    "data": {
                                                        "Concept": {},
                                                        "m_model": {}
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                },
                "data": self.maincanvas.current_signal.tolist()
            }
            
            if self.signal_exists(signal_data):
                QMessageBox.information(self, "Reminder", "Signal already exists!", QMessageBox.StandardButton.Ok)
            else:
                signal_name = f"Signal {self.signal_counter}"
                self.signal_counter += 1
                child = QTreeWidgetItem(self.customizes)
                child.setText(0, signal_name)
                child.setToolTip(0, signal_name)  # Set tooltip
                child.setFlags(child.flags() | QtCore.Qt.ItemFlag.ItemIsEditable)  # Make the item editable
                child.setData(0, QtCore.Qt.ItemDataRole.UserRole, signal_name)  # Store the original name
                self.customizes.addChild(child)
                self.custom_signals[signal_name] = signal_data  # Save the signal data

    @pyqtSlot(QTreeWidgetItem, int)
    def on_tree_item_changed(self, item, column):
        old_name = item.data(column, QtCore.Qt.ItemDataRole.UserRole)
        new_name = item.text(column)
        if item.parent() == self.customizes and old_name in self.custom_signals:
            self.custom_signals[new_name] = self.custom_signals.pop(old_name)
            item.setData(column, QtCore.Qt.ItemDataRole.UserRole, new_name)
            item.setToolTip(0, new_name)  # Update tooltip
        elif item.parent() and item.parent().text(0) == "Imported Signals" and old_name in self.imported_signals:
            self.imported_signals[new_name] = self.imported_signals.pop(old_name)
            item.setData(column, QtCore.Qt.ItemDataRole.UserRole, new_name)
            item.setToolTip(0, new_name)  # Update tooltip

    def delete_tree_item(self, item):
        signal_name = item.text(0)
        if item.parent() == self.customizes:
            if signal_name in self.custom_signals:
                del self.custom_signals[signal_name]
            index = self.customizes.indexOfChild(item)
            if index != -1:
                self.customizes.takeChild(index)
        elif item.parent() and item.parent().text(0) == "Imported Signals":
            if signal_name in self.imported_signals:
                del self.imported_signals[signal_name]
            index = item.parent().indexOfChild(item)
            if index != -1:
                item.parent().takeChild(index)
                
    @pyqtSlot(QtCore.QPoint)
    def on_custom_context_menu(self, point):
        item = self.ui.treeWidget.itemAt(point)
        if item:
            menu = QtWidgets.QMenu(self)
            delete_action = menu.addAction("Delete")
            rename_action = menu.addAction("Rename")

            action = menu.exec(self.ui.treeWidget.mapToGlobal(point))

            if action == delete_action:
                self.delete_tree_item(item)
            elif action == rename_action:
                self.ui.treeWidget.editItem(item)

    def delete_custom_signal(self, item):
        signal_name = item.text(0)
        if signal_name in self.custom_signals:
            del self.custom_signals[signal_name]
        index = self.customizes.indexOfChild(item)
        if index != -1:
            self.customizes.takeChild(index)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = Haptics_App()
    mainWindow.show()
    sys.exit(app.exec())
