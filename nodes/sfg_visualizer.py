#!/usr/bin/env python3

import sys
import rospy

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QPushButton
from symbolic_fact_generation.msg import Facts
from PyQt5.QtGui import QFont

class SFGViz(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.label = QLabel(self)

        font = QFont()
        font.setPointSize(16) # Set the font size to 16 points
        self.label.setFont(font)

        self.label.setText('Waiting for facts...')
        vbox = QVBoxLayout()

        reset_button = QPushButton('Reset')
        reset_button.clicked.connect(self.resetText) # Connect the clicked signal to the resetText slot
        vbox.addWidget(reset_button)

        vbox.addWidget(self.label)
        self.setLayout(vbox)
        self.setGeometry(100, 100, 200, 100)
        self.setWindowTitle('SFG')
        self.show()

        rospy.init_node('sfg_visualiser')
        rospy.Subscriber('/fact_publisher/facts', Facts, self.factsCallback)

    def pretify_text(self, facts):
        s = ''
        for fact in facts:
            s += fact.name + ' ( '
            for value in fact.values:
                s += value + ' '
            s += ')\n'
        return s

    def factsCallback(self, msg):
        self.label.setText(self.pretify_text(msg.facts))

    def resetText(self):
        self.label.setText("Waiting for facts...")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = SFGViz()
    sys.exit(app.exec_())
