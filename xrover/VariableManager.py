from PySide6.QtCore import QObject, QMutex, QMutexLocker, QSettings
from typing import Any


class VariableManager(QObject):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(VariableManager, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        super().__init__()
        self.mutex = QMutex()
        self.file_path = None
        self.cache = {}  # Biến tạm để lưu trữ các giá trị được set

    def load(self, file_path):
        self.file_path = file_path

    def set(self, name: str, value: Any):
        if self.file_path is None:
            return

        with QMutexLocker(self.mutex):
            self.cache[name] = value  # Lưu giá trị vào biến tạm

    def get(self, name: str, default_value=None):
        if self.file_path is None:
            return default_value

        with QMutexLocker(self.mutex):
            # Trước tiên, kiểm tra trong cache
            if name in self.cache:
                return self.cache[name]

            # Nếu không có trong cache, lấy từ file
            settings = QSettings(self.file_path, QSettings.Format.NativeFormat)
            value = settings.value(name, default_value)
            del settings  # Đóng file sau khi tải xong
            return value

    def getBool(self, name: str, default_value=None):
        value = self.get(name, default_value)
        if value == "false" or value == False:
            return False
        elif value == "true" or value == True:
            return True
        return default_value

    def save(self):
        if self.file_path is None:
            return

        with QMutexLocker(self.mutex):
            settings = QSettings(self.file_path, QSettings.Format.NativeFormat)
            for name, value in self.cache.items():
                settings.setValue(name, value)
            settings.sync()  # Ghi tất cả các thay đổi vào file
            del settings  # Đóng file sau khi lưu xong

            # Xóa cache sau khi lưu
            self.cache.clear()


# Create a global instance
instance = VariableManager()
