import tkinter as tk
from tkinter import filedialog

def getFile():
    """ getFile() --> filePath
        Prompt user for json file
    """
    try:
        filetypes = (
        ('json files', '*.json'),
        )
        root = tk.Tk()
        root.withdraw()
        return(filedialog.askopenfilename(filetypes=filetypes))
    except:
        print("fileManagement.getFile(): Error getting filePath")
        return None
    
        

