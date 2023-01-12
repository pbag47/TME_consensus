import asyncio
import gui
import numpy
import qasync
import sys

from agent_class import Agent
from cf_info import init_agents
from PyQt5 import QtCore
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QApplication, QMainWindow
from qwt import QwtPlot, QwtPlotCurve, QwtPlotMarker, QwtPlotGrid, QwtSymbol
from typing import List


class GraphicRepresentation:
    def __init__(self, uav: Agent, checkbox):
        self.uav = uav
        self.marker = None
        self.marker_gr = QwtPlotMarker(self.uav.name)
        self.marker_gr.setValue(0, 0)
        symbol = QwtSymbol(QwtSymbol.XCross)
        symbol.setSize(10, 10)
        self.marker_gr.setSymbol(symbol)
        self.curve = QwtPlotCurve(self.uav.name + ' radius')
        self.curve.setPen(QPen(QtCore.Qt.black))
        self.curve.setStyle(QwtPlotCurve.Lines)
        self.manual = 0
        self.enabled = 0
        self.checkbox = checkbox
        self.update()

    def update(self):
        if self.enabled:
            self.marker_gr.setValue(self.uav.initial_position[0], self.uav.initial_position[1])
            self.marker_gr.setVisible(True)
            if self.marker:
                self.curve.setData([self.uav.initial_position[0], self.marker.x * 10 ** -3],
                                   [self.uav.initial_position[1], self.marker.y * 10 ** -3])
                self.curve.setVisible(True)
            else:
                self.curve.setVisible(False)
        else:
            self.marker_gr.setVisible(False)
            self.curve.setVisible(False)


class QtmMarkersGR:
    def __init__(self, qtm_marker):
        self.marker = qtm_marker
        self.gr = QwtPlotMarker(str(qtm_marker.id))
        symbol = QwtSymbol(QwtSymbol.Ellipse)
        symbol.setSize(10, 10)
        self.gr.setSymbol(symbol)
        self.gr.setValue(self.marker.x * 10 ** -3, self.marker.y * 10 ** -3)

    def erase(self):
        self.gr.detach()
        del self


class Window(QMainWindow, gui.Ui_MainWindow):
    def __init__(self, parent=None, uavs: List[Agent] = None, parameters_filename=None):
        super().__init__(parent)
        self.filename = parameters_filename
        self.setupUi(self)
        self.agents_list = []
        self.manual_agents_list = []
        self.qtm_markers_gr_list = []

        grid = QwtPlotGrid()
        grid.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.DotLine))
        grid.attach(self.plot)

        self.plot.setTitle('Initial position visualizer')
        self.plot.setAxisTitle(QwtPlot.xBottom, 'X (m)')
        self.plot.setAxisTitle(QwtPlot.yLeft, 'Y (m)')
        self.plot.setAxisScale(QwtPlot.xBottom, -2, 2, 0.50)
        self.plot.setAxisScale(QwtPlot.yLeft, -2, 2, 0.50)

        self.x_offset.setRange(-5, 5)
        self.y_offset.setRange(-5, 5)

        self.series = []
        self.enabled_cf_checkbox_list = [self.cb_cf1, self.cb_cf2, self.cb_cf3, self.cb_cf4, self.cb_cf5,
                                         self.cb_cf6, self.cb_cf7, self.cb_cf8, self.cb_cf9, self.cb_cf10]
        self.connectivity_checkbox_list = [self.co_cf1, self.co_cf2, self.co_cf3, self.co_cf4, self.co_cf5,
                                           self.co_cf6, self.co_cf7, self.co_cf8, self.co_cf9, self.co_cf10]
        for uav in uavs:
            cb = [checkbox for checkbox in self.enabled_cf_checkbox_list if checkbox.objectName().endswith(uav.name)]
            self.series.append(GraphicRepresentation(uav, cb[0]))
        for gr in self.series:
            gr.marker_gr.attach(self.plot)
            gr.curve.attach(self.plot)

        self.selected_gr = []
        self.read_parameters_file()
        self.update_combobox()
        self.connect_callbacks()
        self.show()

    def connect_callbacks(self):
        self.cf_choice.currentIndexChanged.connect(self.cf_choice_callback)
        for cb in self.enabled_cf_checkbox_list:
            cb.clicked.connect(self.cf_enabled_callback)
        for cb in self.connectivity_checkbox_list:
            cb.clicked.connect(self.connectivity_changed_callback)
        self.x.valueChanged.connect(self.x_changed_callback)
        self.y.valueChanged.connect(self.y_changed_callback)
        self.z.valueChanged.connect(self.z_changed_callback)
        self.z_takeoff.valueChanged.connect(self.z_takeoff_changed_callback)
        self.x_offset.valueChanged.connect(self.x_offset_changed_callback)
        self.y_offset.valueChanged.connect(self.y_offset_changed_callback)
        self.manual_flight.clicked.connect(self.manual_changed_callback)
        self.valider.clicked.connect(self.submit_callback)

    def read_parameters_file(self):
        with open(self.filename, 'r') as file:
            lines = file.readlines()
            for line in lines:
                str_values = [element.strip() for element in line.split(',')]
                for gr in self.series:
                    if gr.uav.name == str_values[0]:
                        gr.uav.set_initial_position([float(str_values[1]), float(str_values[2]), float(str_values[3])])
                        gr.uav.set_takeoff_height(float(str_values[4]))
                        connectivity = str_values[5]
                        gr.uav.set_consensus_connectivity(connectivity.split(';'))
                        gr.uav.set_xy_consensus_offset([float(str_values[6]), float(str_values[7])])
                        gr.manual = int(str_values[8])
                        gr.enabled = int(str_values[9])

    def cf_choice_callback(self, index):
        self.selected_gr = [gr for gr in self.series if gr.uav.name == self.cf_choice.itemText(index)]
        if self.selected_gr:
            self.update_connectivity_checkbox()
            self.update_parameters_ui()

    def update_parameters_ui(self):
        self.x.setRange(self.selected_gr[0].uav.x_boundaries[0], self.selected_gr[0].uav.x_boundaries[1])
        self.y.setRange(self.selected_gr[0].uav.y_boundaries[0], self.selected_gr[0].uav.y_boundaries[1])
        self.z.setRange(self.selected_gr[0].uav.z_boundaries[0], self.selected_gr[0].uav.z_boundaries[1])
        self.z_takeoff.setRange(self.selected_gr[0].uav.z_boundaries[0], self.selected_gr[0].uav.z_boundaries[1])

        self.x.setValue(self.selected_gr[0].uav.initial_position[0])
        self.y.setValue(self.selected_gr[0].uav.initial_position[1])
        self.z.setValue(self.selected_gr[0].uav.initial_position[2])

        self.z_takeoff.setValue(self.selected_gr[0].uav.takeoff_height)
        self.x_offset.setValue(self.selected_gr[0].uav.xy_consensus_offset[0])
        self.y_offset.setValue(self.selected_gr[0].uav.xy_consensus_offset[1])

    def x_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.initial_position[0] = self.x.value()

    def y_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.initial_position[1] = self.y.value()

    def z_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.initial_position[2] = self.z.value()

    def z_takeoff_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.set_takeoff_height(self.z_takeoff.value())

    def connectivity_changed_callback(self):
        if self.selected_gr:
            connectivity = []
            names = [gr.uav.name for gr in self.series]
            for checkbox in self.connectivity_checkbox_list:
                cb_cf = [name for name in names if checkbox.objectName().endswith(name)]
                if checkbox.isChecked() and cb_cf:
                    connectivity.append(cb_cf[0])
            self.selected_gr[0].uav.set_consensus_connectivity(connectivity)

    def x_offset_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.xy_consensus_offset[0] = self.x_offset.value()

    def y_offset_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].uav.xy_consensus_offset[1] = self.y_offset.value()

    def manual_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].manual = int(self.manual_flight.isChecked())

    def cf_enabled_callback(self):
        for gr in self.series:
            gr.enabled = int(gr.checkbox.isChecked())
        self.update_combobox()

    def update_combobox(self):
        for gr in self.series:
            gr.checkbox.setChecked(gr.enabled)
        names = [gr.uav.name for gr in self.series if gr.enabled]
        self.cf_choice.clear()
        if names:
            self.cf_choice.addItems(names)
            if not self.selected_gr:
                valid_gr = [gr for gr in self.series if gr.enabled]
                self.selected_gr = [valid_gr[0]]
            self.update_connectivity_checkbox()

    def update_connectivity_checkbox(self):
        for checkbox in self.connectivity_checkbox_list:
            checkbox.setEnabled(True)
            checkbox.setChecked(any([checkbox.objectName().endswith(name)
                                     for name in self.selected_gr[0].uav.consensus_connectivity]))
            if checkbox.objectName().endswith(self.selected_gr[0].uav.name):
                checkbox.setEnabled(False)
            else:
                checkbox.setEnabled(True)
        self.manual_flight.setChecked(self.selected_gr[0].manual)

    def submit_callback(self):
        self.update_parameters_file()
        for gr in self.series:
            if gr.enabled:
                self.agents_list.append(gr.uav)
                if gr.manual:
                    self.manual_agents_list.append(gr.uav)
        self.close()

    def update_parameters_file(self):
        text = ['Name, Init_x, Init_y, Init_z, Takeoff_z, Connectivity, Offset_x, Offset_y, Manual, Enabled \n']
        for gr in self.series:
            connectivity = []
            for name in gr.uav.consensus_connectivity:
                connectivity.append(name + ';')
            co = ''.join(connectivity)
            line = gr.uav.name + ', '
            line += str(gr.uav.initial_position[0]) + ', '
            line += str(gr.uav.initial_position[1]) + ', '
            line += str(gr.uav.initial_position[2]) + ', '
            line += str(gr.uav.takeoff_height) + ', '
            line += co[:-1] + ', '
            line += str(gr.uav.xy_consensus_offset[0]) + ', '
            line += str(gr.uav.xy_consensus_offset[1]) + ', '
            line += str(gr.manual) + ', '
            line += str(gr.enabled) + ' \n'
            text.append(line)
        with open(self.filename, 'w') as file:
            file.writelines(text)

    def update_graph(self, packet):
        _, markers = packet.get_3d_markers_no_label()

        for qtm_gr in self.qtm_markers_gr_list:
            qtm_gr.erase()

        self.qtm_markers_gr_list = []
        for marker in markers:
            qtm_gr = QtmMarkersGR(marker)
            qtm_gr.gr.attach(self.plot)
            self.qtm_markers_gr_list.append(qtm_gr)

        # For each enabled UAV, finds the nearest QTM marker
        for gr in self.series:
            if gr.enabled and markers:
                d = [numpy.sqrt((gr.uav.initial_position[0] - (marker.x * 10 ** -3)) ** 2
                                + (gr.uav.initial_position[1] - (marker.y * 10 ** -3)) ** 2)
                     for marker in markers]
                index = d.index(min(d))
                if d[index] <= 0.5:
                    gr.marker = markers[index]
                else:
                    gr.marker = None
            else:
                gr.marker = None
            gr.update()

        self.plot.replot()


if __name__ == '__main__':
    agents_list = init_agents()
    app_test = QApplication(sys.argv)
    filename = 'flight_parameters.txt'
    User_window = Window(uavs=agents_list, parameters_filename=filename)
    q_loop = qasync.QEventLoop(app_test)
    asyncio.set_event_loop(q_loop)
    q_loop.run_forever()
