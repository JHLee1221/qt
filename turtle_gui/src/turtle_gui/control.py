#!/usr/bin/env python3


from turtle_gui.control_widget import ControlWidget

from rqt_gui_py.plugin import Plugin


class Control(Plugin):

    def __init__(self, context):
        super(Control, self).__init__(context)
        self.setObjectName('RQt example')
        self.widget = ControlWidget(context.node)
        serial_number = context.serial_number()
        if serial_number > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the RQt example.')
        self.widget.shutdown_widget()
