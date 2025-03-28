from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QApplication
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont, QColor, QPalette, QTextCharFormat, QTextCursor
import sys
import os
import re

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from Mission_planner.communication.pc_mavlink import LOG_FILE

log_file = os.path.join("./logs", LOG_FILE)

class LogViewer(QWidget):
    def __init__(self):
        super().__init__()

        self.log_file = log_file

        self.setWindowTitle("Mission Log Viewer")
        self.setGeometry(400, 200, 900, 500)

        layout = QVBoxLayout()

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setFont(QFont("Consolas", 11))

        palette = QPalette()
        palette.setColor(QPalette.Base, QColor("#1e1e1e"))
        palette.setColor(QPalette.Text, QColor("#ffffff"))
        self.text_edit.setPalette(palette)
        self.text_edit.setStyleSheet("""
            QTextEdit {
                padding: 0px;
                margin: 0px;
                border: none;
                background-color: #1e1e1e;
                color: #ffffff;
            }
            QScrollBar:vertical {
                background: transparent;
                width: 8px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: rgba(120, 120, 120, 80);  /* Xám mờ */
                border-radius: 4px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background: rgba(180, 180, 180, 150);
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                background: none;
                height: 0px;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
        """)

        layout.setContentsMargins(0, 0, 0, 0)  
        layout.addWidget(self.text_edit)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.load_log)
        self.timer.start(100)

        self.last_content = ""
        self.load_log()

    def highlight_line(self, line):
        cursor = self.text_edit.textCursor()
        format = QTextCharFormat()

        if "ERROR" in line:
            format.setForeground(QColor("#ff5555"))
        elif "WARNING" in line:
            format.setForeground(QColor("#f1fa8c"))
        elif "INFO" in line:
            format.setForeground(QColor("#8be9fd"))
        elif "DEBUG" in line:
            format.setForeground(QColor("#bbbbbb"))
        else:
            format.setForeground(QColor("#ffffff"))

        cursor.insertText(line, format)

    def load_log(self):
        try:
            with open(self.log_file, "r") as f:
                content = f.read()
                if content != self.last_content:    
                    new_part = content[len(self.last_content):]

                    at_bottom = self.text_edit.verticalScrollBar().value() == self.text_edit.verticalScrollBar().maximum()

                    for line in new_part.splitlines():
                        self.highlight_line(line + "\n")

                    if at_bottom:
                        self.text_edit.moveCursor(QTextCursor.End)

                    self.last_content = content
        except FileNotFoundError:
            self.text_edit.setPlainText("Log file not found.")


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     log_file = os.path.join("./logs", LOG_FILE)
#     viewer = LogViewer(log_file)
#     viewer.show()
#     sys.exit(app.exec_())
