#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : rqt_plugin_teleop.py
@Author  : Li_JiaXuan (modified for rqt plugin)
@Date    : 2024-12-04
@Software: rqt / PyCharm
"""

import rosparam
import rospy
import rosgraph.masterapi

from rqt_gui_py.plugin import Plugin
from python_qt_binding import QtWidgets, QtCore


class TeleopModeWidget(QtWidgets.QWidget):
    """
    QWidget that displays the current teleop mode using background color
    and a large text label.
    """

    def __init__(self, parent=None):
        super(TeleopModeWidget, self).__init__(parent)

        # Dictionary mapping teleop_mode values to colors.
        self._color_map = {
            1: "red",
            2: "blue",
            3: "green",
            4: "purple",
            5: "black",
        }

        # Dictionary mapping teleop_mode values to display text.
        self._text_map = {
            1: "State: Operation Mode",
            2: "State: Cartesian Mode",
            3: "State: Spherical Mode",
            4: "State: Locking Mode",
            5: "State: Exit",
        }

        # Create main label.
        self._state_label = QtWidgets.QLabel("State: Unknown")
        self._state_label.setAlignment(QtCore.Qt.AlignCenter)

        # Set a large font size (120 is too large for many screens; adjust if needed).
        font = self._state_label.font()
        font.setPointSize(40)
        self._state_label.setFont(font)

        # change text color to white
        self._state_label.setStyleSheet("QLabel { color : white; }")

        # Layout for the widget.
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self._state_label)
        self.setLayout(layout)

        # Initial background style.
        self._set_background_color("white")

        # QTimer to periodically read the teleop mode parameter from the ROS parameter server.
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update_from_param)
        self._timer.start(100)  # [ms] 0.1 s period

    def _set_background_color(self, color_name: str) -> None:
        """
        Sets the background color of the widget using a Qt style sheet.

        Args:
            color_name (str): Name of the color to use as background.
        """
        # Apply background color to the whole widget.
        self.setStyleSheet("QWidget { background-color: %s; }" % color_name)

    def _update_from_param(self) -> None:
        """
        Periodically called by the QTimer to fetch the teleop mode parameter
        and update the UI.
        """
        try:
            teleop_mode = rosparam.get_param("/hand/control_mode")
        except rosgraph.masterapi.MasterError:
            # Throttled logs are nice, but here we keep it simple to avoid extra state.
            rospy.logwarn_once("Failed to get /hand/control_mode parameter.")
            return
        except KeyError:
            # If the parameter does not exist at all.
            rospy.logwarn_once("Parameter /hand/control_mode does not exist.")
            return

        # Update the UI with the new teleop mode.
        self._update_window(teleop_mode)

    def _update_window(self, teleop_mode: int) -> None:
        """
        Updates the widget background color and the displayed text based on the teleop mode.

        Args:
            teleop_mode (int): The current state to update the widget with.
        """
        color = self._color_map.get(teleop_mode, "white")
        text = self._text_map.get(teleop_mode, "State: Unknown")

        self._set_background_color(color)
        self._state_label.setText(text)

    def shutdown(self) -> None:
        """
        Stops timers and performs clean-up when the plugin is shut down.
        """
        if self._timer.isActive():
            self._timer.stop()
        self._timer.deleteLater()


class TeleopModeRqtPlugin(Plugin):
    """
    rqt plugin wrapper that embeds TeleopModeWidget into the rqt framework.
    """

    def __init__(self, context):
        """
        Initializes the plugin and adds the widget to the rqt context.

        Args:
            context: rqt context passed by the framework.
        """
        super(TeleopModeRqtPlugin, self).__init__(context)
        self.setObjectName("TeleopModeRqtPlugin")

        # It is recommended not to call rospy.init_node() inside a plugin.
        # rqt already initializes the ROS node.

        # Create the main widget.
        self._widget = TeleopModeWidget()

        # Set the window title when shown as a standalone widget.
        self._widget.setWindowTitle("Teleop Mode Window")

        # If multiple instances of the plugin are opened, append the instance number.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + " (%d)" % context.serial_number())

        # Add widget to the user interface.
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        """
        Called when the plugin is being shut down. Use this to clean up
        resources such as timers, subscribers, etc.
        """
        if self._widget is not None:
            self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        """
        Saves intrinsic configuration to the plugin-specific or instance-specific
        configuration files.

        Args:
            plugin_settings: Global plugin settings.
            instance_settings: Settings specific to this instance of the plugin.
        """
        # Currently no settings to save.
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """
        Restores intrinsic configuration from the plugin-specific or
        instance-specific configuration files.

        Args:
            plugin_settings: Global plugin settings.
            instance_settings: Settings specific to this instance of the plugin.
        """
        # Currently no settings to restore.
        pass
