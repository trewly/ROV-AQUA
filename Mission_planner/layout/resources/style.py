canvas_button_style="""
            QPushButton {
                background-color: #3E5A60;  /* Màu nền xanh đậm */
                color: #F0EED4;  /* Màu chữ kem */
                font-size: 16px;
                font-weight: bold;
                padding: 10px 10px;
                border: 1px solid black;
            }
            QPushButton:hover {
                background-color: #4A6D75; /* Màu sáng hơn khi hover */
            }
            QPushButton:pressed {
                background-color: #2D4146; /* Màu đậm hơn khi bấm */
            }
        """

control_button_style="""
            QPushButton {
                background-color: #3E5A60;  /* Màu nền xanh đậm */
                color: #F0EED4;  /* Màu chữ kem */
                font-size: 12px;
                font-weight: bold;
                padding: 8px 8px;
                border: 1px solid black;
            }
            QPushButton:hover {
                background-color: #4A6D75; /* Màu sáng hơn khi hover */
            }
            QPushButton:pressed {
                background-color: #2D4146; /* Màu đậm hơn khi bấm */
            }
        """

normal_text_info_style="""
            QLabel {
                color: #F3F3E0;  /* Màu chữ kem */
                font-size: 20px;
                font-weight: bold;
            }
"""

setting_button_style="""
    QPushButton {
        background-color: #F3F3E0;  
        color: #000000;  
        font-size: 18px;
        font-weight: medium-bold;
        border-radius: 5px;
        border: 1px solid black;
    }
    QPushButton:hover {
        background-color: #2C3333; 
        color: #E7F6F2;  
    }
"""

warning_label_style = """
    QLabel {
        color: #FFA500;  /* Màu cam cảnh báo */
        font-size: 17px;
        font-weight: bold;
        background-color: rgba(255, 165, 0, 0.1); /* Nền cam nhạt */
        border-left: 4px solid #FFA500;
        padding: 5px;
        border-radius: 3px;
    }
"""

success_label_style = """
    QLabel {
        color: #4CAF50;  /* Màu xanh lá thành công */
        font-size: 17px;
        font-weight: bold;
        background-color: rgba(76, 175, 80, 0.1); /* Nền xanh nhạt */
        border-left: 4px solid #4CAF50;
        padding: 5px;
        border-radius: 3px;
    }
"""
