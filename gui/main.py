from PySide6.QtWidgets import QFileDialog
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtCore import QUrl, QObject, Signal, Slot
from PySide6.QtGui import QGuiApplication
import sys
import yaml
import os


class ConfigManager(QObject):
    """Backend manager for config operations exposed to QML."""
    
    statusUpdated = Signal(str)
    configLoaded = Signal()
    
    def __init__(self, parent_window=None):
        super().__init__()
        self.config_data = {}
        self.current_file_path = None
        self.parent_window = parent_window
    
    @Slot()
    def loadConfig(self):
        """Load config from YAML file."""
        try:
            # Get the main app window for dialog parent
            app = QGuiApplication.instance()
            parent_window = app.focusWidget() if app.focusWidget() else None
            
            file_path, _ = QFileDialog.getOpenFileName(
                parent_window, "Open Config", "", "YAML Files (*.yaml *.yml)"
            )
            
            if not file_path:
                return
            
            with open(file_path, "r") as f:
                self.config_data = yaml.safe_load(f) or {}
            
            self.current_file_path = file_path
            self.statusUpdated.emit(f"Loaded: {file_path}")
            self.configLoaded.emit()
        except Exception as e:
            self.statusUpdated.emit(f"Error loading config: {e}")
    
    @Slot()
    def saveConfig(self):
        """Save config to YAML file."""
        if not self.current_file_path:
            self.statusUpdated.emit("No file loaded")
            return
        
        try:
            with open(self.current_file_path, "w") as f:
                yaml.safe_dump(self.config_data, f, sort_keys=False)
            
            self.statusUpdated.emit(f"Saved: {self.current_file_path}")
        except Exception as e:
            self.statusUpdated.emit(f"Error saving config: {e}")
    
    @Slot(str, str, str)
    def setField(self, section, field, value):
        """Set a configuration field value."""
        if section not in self.config_data:
            self.config_data[section] = {}
        
        # Convert string values to appropriate types
        if value.lower() in ['true', 'false']:
            self.config_data[section][field] = value.lower() == 'true'
        elif value.isdigit():
            self.config_data[section][field] = int(value)
        else:
            self.config_data[section][field] = value
    
    @Slot(str)
    def saveApplicationLayerConfig(self, appLayerJson):
        """Save Application Layer configuration to a YAML file."""
        try:
            import json
            
            print("DEBUG: saveApplicationLayerConfig called")
            
            # Parse the JSON data from QML
            app_layer_data = json.loads(appLayerJson)
            print(f"DEBUG: Parsed app layer data: {app_layer_data}")
            
            # Show save dialog
            print(f"DEBUG: Parent window: {self.parent_window}")
            file_path, _ = QFileDialog.getSaveFileName(
                self.parent_window, 
                "Save Application Layer Config", 
                "ApplicationLayer.yaml",
                "YAML Files (*.yaml *.yml)"
            )
            
            print(f"DEBUG: File path selected: {file_path}")
            
            if not file_path:
                print("DEBUG: Save cancelled by user")
                self.statusUpdated.emit("Save cancelled")
                return
            
            # Create the ApplicationLayer section for the config
            app_layer_config = {
                "ApplicationLayer": {
                    "enabled": app_layer_data.get("applicationLayerEnabled", False),
                    "dataFields": app_layer_data.get("selectedDataFields", []),
                    "subscriptions": app_layer_data.get("subscriptions", [])
                }
            }
            
            # Save to file
            with open(file_path, "w") as f:
                yaml.safe_dump(app_layer_config, f, sort_keys=False, default_flow_style=False)
            
            print(f"DEBUG: File saved successfully to {file_path}")
            self.statusUpdated.emit(f"Application Layer config saved to: {file_path}")
        except Exception as e:
            print(f"DEBUG: Error in saveApplicationLayerConfig: {e}")
            import traceback
            traceback.print_exc()
            self.statusUpdated.emit(f"Error saving Application Layer config: {e}")


def main():
    app = QGuiApplication(sys.argv)
    
    # Create QML engine
    engine = QQmlApplicationEngine()
    
    # Load QML file first to get the root window
    qml_file = os.path.join(os.path.dirname(__file__), "main.qml")
    engine.load(QUrl.fromLocalFile(qml_file))
    
    if not engine.rootObjects():
        sys.exit(-1)
    
    # Get the root window for dialog parent
    root_window = engine.rootObjects()[0]
    
    # Create config manager with the root window as parent
    config_manager = ConfigManager(parent_window=root_window)
    
    # Register config manager to QML
    engine.rootContext().setContextProperty("configManager", config_manager)
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
