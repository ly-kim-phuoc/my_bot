import tkinter as tk

window = tk.Tk()
greeting = tk.Label(
    text="Hello, Tkinter",
    foreground="white",  # Set the text color to white
    background="black",  # Set the background color to black
    width=20,
    height=10
)
greeting.pack()

entry =tk.Entry()
entry.pack()
name=entry.get()

button = tk.Button(
    text="Click me!",
    width=25,
    height=5,
    bg="blue",
    fg="yellow",
)
button.pack()


window.mainloop()