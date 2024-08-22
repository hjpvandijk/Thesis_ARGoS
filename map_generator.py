import tkinter as tk
from tkinter import simpledialog

canvas_width = 1000
canvas_height = 1000

class DrawApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Drawing Boxes")

        self.canvas = tk.Canvas(root, bg="white", width=canvas_width, height=canvas_height)
        self.canvas.pack()

        self.rect = None
        self.start_x = None
        self.start_y = None

        self.box_counter = 0
        self.boxes = []

        self.draw_grid()

        self.canvas.bind("<Button-1>", self.on_button_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)
    
    def draw_grid(self):
        # Draw grid lines every 50 pixels (1 unit in the arena coordinates)
        for i in range(0, canvas_width, (int) (canvas_width/10)):
            # Vertical lines
            self.canvas.create_line(i, 0, i, canvas_width, fill="lightgray")
            # Horizontal lines
            self.canvas.create_line(0, i, canvas_height, i, fill="lightgray")

        # Draw the center lines
        self.canvas.create_line(canvas_width/2, 0, canvas_width/2, canvas_width, fill="black")  # Vertical center line
        self.canvas.create_line(0, canvas_height/2, canvas_height, canvas_height/2, fill="black")  # Horizontal center line

    def on_button_press(self, event):
        # Save the starting point
        self.start_x = event.x
        self.start_y = event.y

        # Create rectangle
        self.rect = self.canvas.create_rectangle(self.start_x, self.start_y, self.start_x, self.start_y, outline="black")

    def on_mouse_drag(self, event):
        # Expand rectangle as the mouse is dragged
        self.canvas.coords(self.rect, self.start_x, self.start_y, event.x, event.y)

    def on_button_release(self, event):
        # Finalize rectangle position and size
        end_x, end_y = event.x, event.y

        print(self.start_x, self.start_y)
        print(end_x, end_y)

        # Calculate width and height
        width = abs(end_x - self.start_x)/2
        height = abs(end_y - self.start_y)/2

        # Calculate center position
        center_x = (self.start_x + end_x) / 2
        center_y = (self.start_y + end_y) / 2

        arena_x = (center_x - canvas_width/2) / (canvas_width/10)
        arena_y = -(center_y - canvas_height/2) / (canvas_height/10)  # Inverting y-axis to match the typical coordinate system



        # Box size and position in XML format
        box_id = f"box_{self.box_counter}"
        xml_size = f"{height/50},{width/50},0.5"
        # xml_position = f"{(250-center_y)/50},{-(center_x-250)/50},0"
        xml_position = f"{arena_y},{-arena_x},0"

        xml = f'''<box id="{box_id}" size="{xml_size}" movable="false">
    <body position="{xml_position}" orientation="0,0,0"/>
</box>'''

        self.boxes.append(xml)
        print(xml)

        self.box_counter += 1

    def get_xml(self):
        return "\n".join(self.boxes)

def main():
    root = tk.Tk()
    app = DrawApp(root)
    root.mainloop()

    # Print out the final XML
    print("Final XML Output:\n")
    print(app.get_xml())

if __name__ == "__main__":
    main()
