import tkinter as tk


# the class for creating instances of menu buttons
class menu_button():
    def __init__(self, background, button_text, button_font, button_fg, button_bg, button_xcor, button_ycor, button_func):
        self.button = tk.Label(master = background, text = button_text, font = button_font, fg = button_fg, bg = button_bg)
        self.button.place(x = button_xcor, y = button_ycor, anchor = "center")
        self.button.bind("<Enter>", lambda event, button = self.button: button.configure(font = "{} {} bold".format(button["font"].split(" ")[0], int(button["font"].split(" ")[1]) + 5)))
        self.button.bind("<Leave>", lambda event, button = self.button: button.configure(font=button_font))
        self.button.bind("<Button-1>", lambda event: button_func(event))


# the class for creating instances of menu labels
class menu_label():
    def __init__(self, background, label_text, label_font, label_fg, label_bg, label_xcor, label_ycor):
        self.label = tk.Label(master = background, text = label_text, font = label_font, fg = label_fg, bg = label_bg)
        self.label.place(x=label_xcor, y=label_ycor, anchor = "center")