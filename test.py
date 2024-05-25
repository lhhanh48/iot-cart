import tkinter as tk
import threading
import time

def update_logs(text_widget):
    # Example: Update logs every second with the current time
    while True:
        current_time = time.strftime("%H:%M:%S")
        text_widget.config(state="normal")
        text_widget.insert(tk.END, current_time + "\n")
        text_widget.config(state="disabled")
        text_widget.see(tk.END)  # Scroll to the end
        time.sleep(1)

def display_logs():
    root = tk.Tk()
    root.title("Log Viewer")

    text = tk.Text(root)
    text.pack(expand=True, fill="both")
    text.configure(state="disabled")

    # Start a thread to update the logs
    update_thread = threading.Thread(target=update_logs, args=(text,))
    update_thread.daemon = True  # Daemonize the thread
    update_thread.start()

    root.mainloop()

if __name__ == "__main__":
    display_logs()

    # Code here will execute after the main window is closed
    print("Main window closed. Continuing with other activities...")

