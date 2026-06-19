class Configs:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Configs, cls).__new__(cls)
            # Initialize any variables that you need
            cls._instance.value = "Default Configuration"
        return cls._instance

    def get_value(self):
        return self._instance.value

    def set_value(self, value):
        self._instance.value = value