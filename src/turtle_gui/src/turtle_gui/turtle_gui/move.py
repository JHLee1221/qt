from turtle_gui.form import WindowClass

from rqt_gui.py.plugin import Plugin


class Move(Plugin):
    def __init__(self, context):
        super(Move, self).__init__(context)
        self.setOblectName('Form')
        self.widget = WindowClass(context.node)
        serial_number = context.serial_number()
        if serial_number > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print("Shutdown the Form.")
        self.widget.shutdown_widget()
