import os
import tkinter as tk
from python_scripts.gui import PneumaticGUI

def main():
    root = tk.Tk()
    base_path = os.path.dirname(os.path.abspath(__file__))
    icon_filename = "icon.png"
    icon_path = os.path.join(base_path, icon_filename)
    if os.path.exists(icon_path):
        try:
            icon_image = tk.PhotoImage(file=icon_path)
            root.iconphoto(True, icon_image)
        except Exception as e:
            print(f"Error loading icon: {e}")
    else:
        print(f"Warning: {icon_filename} not found in {base_path}")
    app = PneumaticGUI(root)
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()

if __name__ == "__main__":
    main()