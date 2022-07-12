# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'flyami_viewer.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1376, 1038)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.ca_img_frame = QtWidgets.QFrame(self.frame)
        self.ca_img_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.ca_img_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.ca_img_frame.setObjectName("ca_img_frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.ca_img_frame)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.frame_2 = QtWidgets.QFrame(self.ca_img_frame)
        self.frame_2.setMinimumSize(QtCore.QSize(521, 871))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.ca_img_view = GraphicsView(self.frame_2)
        self.ca_img_view.setMinimumSize(QtCore.QSize(521, 451))
        self.ca_img_view.setObjectName("ca_img_view")
        self.verticalLayout_4.addWidget(self.ca_img_view)
        self.frame_3 = QtWidgets.QFrame(self.frame_2)
        self.frame_3.setMinimumSize(QtCore.QSize(501, 191))
        self.frame_3.setMaximumSize(QtCore.QSize(16777215, 191))
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.frame_3)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_2 = QtWidgets.QLabel(self.frame_3)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 0, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.frame_3)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.frame_3)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 0, 3, 1, 1)
        self.hist_view = GraphicsView(self.frame_3)
        self.hist_view.setMaximumSize(QtCore.QSize(201, 191))
        self.hist_view.setObjectName("hist_view")
        self.gridLayout_2.addWidget(self.hist_view, 1, 0, 2, 1)
        self.gamma_view = GraphicsView(self.frame_3)
        self.gamma_view.setMaximumSize(QtCore.QSize(211, 191))
        self.gamma_view.setObjectName("gamma_view")
        self.gridLayout_2.addWidget(self.gamma_view, 1, 1, 2, 1)
        self.gamma_slider = QtWidgets.QSlider(self.frame_3)
        self.gamma_slider.setOrientation(QtCore.Qt.Vertical)
        self.gamma_slider.setObjectName("gamma_slider")
        self.gridLayout_2.addWidget(self.gamma_slider, 1, 2, 2, 1)
        self.ca_fps_disp = QtWidgets.QLineEdit(self.frame_3)
        self.ca_fps_disp.setObjectName("ca_fps_disp")
        self.gridLayout_2.addWidget(self.ca_fps_disp, 1, 3, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(138, 112, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem, 2, 3, 1, 1)
        self.verticalLayout_4.addWidget(self.frame_3)
        self.muscle_profile_frame = QtWidgets.QFrame(self.frame_2)
        self.muscle_profile_frame.setMinimumSize(QtCore.QSize(461, 211))
        self.muscle_profile_frame.setMaximumSize(QtCore.QSize(16777215, 211))
        self.muscle_profile_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.muscle_profile_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.muscle_profile_frame.setObjectName("muscle_profile_frame")
        self.gridLayout = QtWidgets.QGridLayout(self.muscle_profile_frame)
        self.gridLayout.setObjectName("gridLayout")
        self.label_8 = QtWidgets.QLabel(self.muscle_profile_frame)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 0, 0, 1, 1)
        self.componentsView = QtWidgets.QListView(self.muscle_profile_frame)
        self.componentsView.setMinimumSize(QtCore.QSize(331, 171))
        self.componentsView.setObjectName("componentsView")
        self.gridLayout.addWidget(self.componentsView, 0, 1, 6, 1)
        self.profileName = QtWidgets.QLineEdit(self.muscle_profile_frame)
        self.profileName.setObjectName("profileName")
        self.gridLayout.addWidget(self.profileName, 1, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.muscle_profile_frame)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 2, 0, 1, 1)
        self.modelSelectBox = QtWidgets.QComboBox(self.muscle_profile_frame)
        self.modelSelectBox.setObjectName("modelSelectBox")
        self.gridLayout.addWidget(self.modelSelectBox, 3, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.muscle_profile_frame)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 4, 0, 1, 1)
        self.profileSelectBox = QtWidgets.QComboBox(self.muscle_profile_frame)
        self.profileSelectBox.setObjectName("profileSelectBox")
        self.gridLayout.addWidget(self.profileSelectBox, 5, 0, 1, 1)
        self.verticalLayout_4.addWidget(self.muscle_profile_frame)
        self.horizontalLayout.addWidget(self.frame_2)
        self.frame_4 = QtWidgets.QFrame(self.ca_img_frame)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_4)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.frame_6 = QtWidgets.QFrame(self.frame_4)
        self.frame_6.setMinimumSize(QtCore.QSize(771, 801))
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.frame_6)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.b_viewer = GraphicsView(self.frame_6)
        self.b_viewer.setObjectName("b_viewer")
        self.gridLayout_4.addWidget(self.b_viewer, 0, 0, 1, 1)
        self.i_viewer = GraphicsView(self.frame_6)
        self.i_viewer.setObjectName("i_viewer")
        self.gridLayout_4.addWidget(self.i_viewer, 0, 1, 1, 1)
        self.iii_viewer = GraphicsView(self.frame_6)
        self.iii_viewer.setObjectName("iii_viewer")
        self.gridLayout_4.addWidget(self.iii_viewer, 1, 0, 1, 1)
        self.hg_viewer = GraphicsView(self.frame_6)
        self.hg_viewer.setObjectName("hg_viewer")
        self.gridLayout_4.addWidget(self.hg_viewer, 1, 1, 1, 1)
        self.verticalLayout_3.addWidget(self.frame_6)
        self.frame_5 = QtWidgets.QFrame(self.frame_4)
        self.frame_5.setMinimumSize(QtCore.QSize(751, 131))
        self.frame_5.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.frame_5)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.frame_7 = QtWidgets.QFrame(self.frame_5)
        self.frame_7.setMinimumSize(QtCore.QSize(201, 111))
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.frame_7)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.label = QtWidgets.QLabel(self.frame_7)
        self.label.setObjectName("label")
        self.gridLayout_5.addWidget(self.label, 0, 0, 1, 2)
        self.line = QtWidgets.QFrame(self.frame_7)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout_5.addWidget(self.line, 1, 0, 1, 2)
        self.trig_mode_combo = QtWidgets.QComboBox(self.frame_7)
        self.trig_mode_combo.setObjectName("trig_mode_combo")
        self.gridLayout_5.addWidget(self.trig_mode_combo, 2, 0, 1, 2)
        self.label_11 = QtWidgets.QLabel(self.frame_7)
        self.label_11.setObjectName("label_11")
        self.gridLayout_5.addWidget(self.label_11, 3, 0, 1, 1)
        self.z_thresh_spin = QtWidgets.QDoubleSpinBox(self.frame_7)
        self.z_thresh_spin.setObjectName("z_thresh_spin")
        self.gridLayout_5.addWidget(self.z_thresh_spin, 3, 1, 1, 1)
        self.horizontalLayout_2.addWidget(self.frame_7)
        self.frame_8 = QtWidgets.QFrame(self.frame_5)
        self.frame_8.setMinimumSize(QtCore.QSize(211, 111))
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.frame_8)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_4 = QtWidgets.QLabel(self.frame_8)
        self.label_4.setObjectName("label_4")
        self.gridLayout_6.addWidget(self.label_4, 0, 0, 1, 1)
        self.pattern_mode_combo = QtWidgets.QComboBox(self.frame_8)
        self.pattern_mode_combo.setObjectName("pattern_mode_combo")
        self.gridLayout_6.addWidget(self.pattern_mode_combo, 0, 1, 1, 2)
        self.label_12 = QtWidgets.QLabel(self.frame_8)
        self.label_12.setObjectName("label_12")
        self.gridLayout_6.addWidget(self.label_12, 1, 0, 1, 2)
        self.closed_loop_gain_spin = QtWidgets.QDoubleSpinBox(self.frame_8)
        self.closed_loop_gain_spin.setObjectName("closed_loop_gain_spin")
        self.gridLayout_6.addWidget(self.closed_loop_gain_spin, 1, 2, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.frame_8)
        self.label_13.setObjectName("label_13")
        self.gridLayout_6.addWidget(self.label_13, 2, 0, 1, 2)
        self.open_loop_gain_spin = QtWidgets.QDoubleSpinBox(self.frame_8)
        self.open_loop_gain_spin.setObjectName("open_loop_gain_spin")
        self.gridLayout_6.addWidget(self.open_loop_gain_spin, 2, 2, 1, 1)
        self.horizontalLayout_2.addWidget(self.frame_8)
        self.frame_9 = QtWidgets.QFrame(self.frame_5)
        self.frame_9.setMinimumSize(QtCore.QSize(321, 111))
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame_9)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_9 = QtWidgets.QLabel(self.frame_9)
        self.label_9.setObjectName("label_9")
        self.gridLayout_3.addWidget(self.label_9, 0, 0, 1, 1)
        self.z_score_disp = QtWidgets.QLineEdit(self.frame_9)
        self.z_score_disp.setObjectName("z_score_disp")
        self.gridLayout_3.addWidget(self.z_score_disp, 0, 1, 1, 2)
        self.label_10 = QtWidgets.QLabel(self.frame_9)
        self.label_10.setObjectName("label_10")
        self.gridLayout_3.addWidget(self.label_10, 1, 0, 1, 2)
        self.record_btn = QtWidgets.QPushButton(self.frame_9)
        self.record_btn.setObjectName("record_btn")
        self.gridLayout_3.addWidget(self.record_btn, 1, 2, 1, 1)
        self.trigger_disp = QtWidgets.QLineEdit(self.frame_9)
        self.trigger_disp.setObjectName("trigger_disp")
        self.gridLayout_3.addWidget(self.trigger_disp, 2, 0, 1, 2)
        self.stop_btn = QtWidgets.QPushButton(self.frame_9)
        self.stop_btn.setObjectName("stop_btn")
        self.gridLayout_3.addWidget(self.stop_btn, 2, 2, 1, 1)
        self.horizontalLayout_2.addWidget(self.frame_9)
        self.verticalLayout_3.addWidget(self.frame_5)
        self.horizontalLayout.addWidget(self.frame_4)
        self.verticalLayout_2.addWidget(self.ca_img_frame)
        self.verticalLayout.addWidget(self.frame)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Flyami Live"))
        self.label_2.setText(_translate("MainWindow", "Image histogram"))
        self.label_3.setText(_translate("MainWindow", "Gamma"))
        self.label_5.setText(_translate("MainWindow", "fps:"))
        self.label_8.setText(_translate("MainWindow", "profile:"))
        self.label_6.setText(_translate("MainWindow", "load model"))
        self.label_7.setText(_translate("MainWindow", "load profile"))
        self.label.setText(_translate("MainWindow", "Set trigger mode:"))
        self.label_11.setText(_translate("MainWindow", "z-threshold:"))
        self.label_4.setText(_translate("MainWindow", "Pattern mode:"))
        self.label_12.setText(_translate("MainWindow", "closed loop gain:"))
        self.label_13.setText(_translate("MainWindow", "open loop gain:"))
        self.label_9.setText(_translate("MainWindow", "Recording:"))
        self.label_10.setText(_translate("MainWindow", "Number of triggers:"))
        self.record_btn.setText(_translate("MainWindow", "start recording"))
        self.stop_btn.setText(_translate("MainWindow", "stop recording"))

from pyqtgraph import GraphicsView