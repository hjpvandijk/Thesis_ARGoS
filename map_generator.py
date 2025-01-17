import tkinter as tk
from tkinter import simpledialog, filedialog
from PIL import Image, ImageTk
import math

canvas_width = 1400
canvas_height = 1400
meter_pixels = 100

class DrawApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Drawing Boxes")

        self.canvas = tk.Canvas(root, bg="white", width=canvas_width, height=canvas_height)
        self.canvas.pack()

        self.rect = None
        self.line = None
        self.start_x = None
        self.start_y = None
        self.end_x = None
        self.end_y = None
        self.is_drawing_line = True
        self.set_box = False

        self.box_counter = 0
        self.boxes = []

        self.scale_factor = 1.0

        self.draw_grid()

        self.canvas.bind("<Button-1>", self.on_button_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

        self.image = None
        self.image_id = None
        self.image_angle = 0
        self.image_x = 0
        self.image_y = 0

        # Add a button to load the background image
        self.load_image_button = tk.Button(root, text="Load Background Image", command=self.load_image)
        self.load_image_button.pack()

        # Add a scale to adjust the image size
        self.scale = tk.Scale(root, from_=0.1, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, label="Scale Image", command=self.scale_image)
        self.scale.set(1.0)
        self.scale.pack()

        # Add buttons to move the image
        self.move_left_button = tk.Button(root, text="Move Left", command=lambda: self.move_image(-10, 0))
        self.move_left_button.pack(side=tk.LEFT)
        self.move_right_button = tk.Button(root, text="Move Right", command=lambda: self.move_image(10, 0))
        self.move_right_button.pack(side=tk.LEFT)
        self.move_up_button = tk.Button(root, text="Move Up", command=lambda: self.move_image(0, -10))
        self.move_up_button.pack(side=tk.LEFT)
        self.move_down_button = tk.Button(root, text="Move Down", command=lambda: self.move_image(0, 10))
        self.move_down_button.pack(side=tk.LEFT)

        # Add buttons to rotate the image
        self.rotate_left_button = tk.Button(root, text="Rotate Left", command=lambda: self.rotate_image(-10))
        self.rotate_left_button.pack(side=tk.LEFT)
        self.rotate_right_button = tk.Button(root, text="Rotate Right", command=lambda: self.rotate_image(10))
        self.rotate_right_button.pack(side=tk.LEFT)

    def draw_grid(self):
        # Draw grid lines every meter
        for i in range(0, canvas_width, meter_pixels):
            # Vertical lines
            self.canvas.create_line(i, 0, i, canvas_width, fill="lightgray")
            # Horizontal lines
            self.canvas.create_line(0, i, canvas_height, i, fill="lightgray")

        # Draw the center lines
        self.canvas.create_line(canvas_width/2, 0, canvas_width/2, canvas_width, fill="black")  # Vertical center line
        self.canvas.create_line(0, canvas_height/2, canvas_height, canvas_height/2, fill="black")  # Horizontal center line

    def load_image(self):
        # file_path = filedialog.askopenfilename()
        file_path = "dunder_mifflin.png"
        if file_path:
            self.original_image = Image.open(file_path)
            self.display_image(1.0)

    def display_image(self, scale):
        if self.original_image:
            width, height = self.original_image.size
            aspect_ratio = width / height

            if aspect_ratio > 1:
                new_width = int(canvas_width * scale)
                new_height = int(new_width / aspect_ratio)
            else:
                new_height = int(canvas_height * scale)
                new_width = int(new_height * aspect_ratio)

            image = self.original_image.resize((new_width, new_height), Image.ANTIALIAS)
            image = image.rotate(self.image_angle, expand=True)
            self.image = ImageTk.PhotoImage(image)

            if self.image_id:
                self.canvas.delete(self.image_id)
            self.image_id = self.canvas.create_image(self.image_x, self.image_y, anchor=tk.NW, image=self.image)

    def scale_image(self, scale_value):
        scale = float(scale_value)
        self.display_image(scale)

    def move_image(self, dx, dy):
        self.image_x += dx
        self.image_y += dy
        self.display_image(self.scale.get())

    def rotate_image(self, angle):
        self.image_angle += angle
        self.display_image(self.scale.get())

    def on_button_press(self, event):
        if not self.set_box:
            self.is_drawing_line = True
            # Save the starting point
            self.start_x = event.x
            self.start_y = event.y

            # Create line
            self.line = self.canvas.create_line(self.start_x, self.start_y, self.start_x, self.start_y, fill="black")
            self.set_box = True
        else:
            # Set the width and create the box
            self.set_width_and_create_box(event.x, event.y)

    def on_mouse_drag(self, event):
        if self.is_drawing_line:
            # Expand line as the mouse is dragged
            self.canvas.coords(self.line, self.start_x, self.start_y, event.x, event.y)

    def on_button_release(self, event):
        if self.is_drawing_line:
            # Save the ending point
            self.end_x = event.x
            self.end_y = event.y
            if (self.start_x, self.start_y) != (self.end_x, self.end_y):
                self.is_drawing_line = False
            else:
                self.set_box = False

    def set_width_and_create_box(self, width_x, width_y):
        if self.line:
            # Calculate the width based on the mouse position
            width = abs(((self.end_x - self.start_x) * (width_y - self.start_y) - (self.end_y - self.start_y) * (width_x - self.start_x)) / ((self.end_x - self.start_x)**2 + (self.end_y - self.start_y)**2)**0.5)
            # Calculate the direction vector of the line
            dx = self.end_x - self.start_x
            dy = self.end_y - self.start_y
            length = (dx**2 + dy**2)**0.5

            # Normalize the direction vector
            dx /= length
            dy /= length

            # Calculate the perpendicular vector
            perp_dx = -dy * width / 2
            perp_dy = dx * width / 2

            # Determine if (width_x, width_y) is on the left or right side of the line
            if (width_x - self.start_x) * dy - (width_y - self.start_y) * dx < 0:
                perp_dx = -perp_dx
                perp_dy = -perp_dy

            # Calculate the four corners of the box with the line as one of its edges
            x1 = self.start_x
            y1 = self.start_y
            x2 = self.end_x
            y2 = self.end_y
            x3 = self.end_x - perp_dx * 2
            y3 = self.end_y - perp_dy * 2
            x4 = self.start_x - perp_dx * 2
            y4 = self.start_y - perp_dy * 2

            # Create the rectangle
            self.rect = self.canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4, outline="black", fill="")

            # Calculate center position
            center_x = (x1 + x2 + x3 + x4) / 4
            center_y = (y1 + y2 + y3 + y4) / 4

            arena_x = (center_x - canvas_width/2) / meter_pixels
            arena_y = -(center_y - canvas_height/2) / meter_pixels  # Inverting y-axis to match the typical coordinate system

            # Box size and position in XML format
            box_id = f"box_{self.box_counter}"
            xml_size = f"{width/meter_pixels},{length/meter_pixels},0.5"
            xml_position = f"{arena_y},{-arena_x},0"

            # Calculate the orientation angle in radians
            angle = -math.degrees(math.atan2(dy, dx))
            xml_orientation = f"{angle},0,0"

            xml = f'''<box id="{box_id}" size="{xml_size}" movable="false">
    <body position="{xml_position}" orientation="{xml_orientation}"/>
</box>'''

            self.boxes.append(xml)
            print(xml)

            self.box_counter += 1

        # Reset for the next box
        self.line = None
        self.set_box = False

    def get_xml(self):
        return "\n".join(self.boxes)

    def on_mouse_wheel(self, event):
        print("Mouse wheel event")
        if event.delta > 0:
            self.scale_factor *= 1.1
        else:
            self.scale_factor /= 1.1
        self.canvas.scale("all", 0, 0, self.scale_factor, self.scale_factor)
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

def main():
    root = tk.Tk()
    app = DrawApp(root)
    root.mainloop()

    # Print out the final XML
    print("Final XML Output:\n")
    print(app.get_xml())

if __name__ == "__main__":
    main()