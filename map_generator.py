import tkinter as tk
from tkinter import simpledialog, filedialog
from PIL import Image, ImageTk
import math

canvas_width = 2000*2
canvas_height = 1020*2
meter_pixels = 100*2

class DrawApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Drawing Shapes")

        self.canvas = tk.Canvas(root, bg="white", width=canvas_width, height=canvas_height)
        self.canvas.pack()

        self.rect = None
        self.line = None
        self.circle = None
        self.start_x = None
        self.start_y = None
        self.end_x = None
        self.end_y = None
        self.is_drawing_line = False
        self.is_drawing_box = False
        self.set_box = False
        self.set_circle = False

        self.box_counter = 0
        self.circle_counter = 0
        self.shapes = []
        self.lines = []
        self.shape_ids = []

        self.scale_factor = 1.0

        self.draw_grid()

        self.canvas.bind("<Button-1>", self.on_button_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)

        self.image = None
        self.image_id = None
        self.image_angle = 0
        self.image_x = 0
        self.image_y = 0

        self.load_image()

        self.scale = tk.Scale(root, from_=0.1, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, label="Scale Image", command=self.scale_image)
        self.scale.set(1.0)
        self.scale.pack()

        self.move_left_button = tk.Button(root, text="Move Left", command=lambda: self.move_image(-10, 0))
        self.move_left_button.pack(side=tk.LEFT)
        self.move_right_button = tk.Button(root, text="Move Right", command=lambda: self.move_image(10, 0))
        self.move_right_button.pack(side=tk.LEFT)
        self.move_up_button = tk.Button(root, text="Move Up", command=lambda: self.move_image(0, -10))
        self.move_up_button.pack(side=tk.LEFT)
        self.move_down_button = tk.Button(root, text="Move Down", command=lambda: self.move_image(0, 10))
        self.move_down_button.pack(side=tk.LEFT)

        self.rotate_left_button = tk.Button(root, text="Rotate Left", command=lambda: self.rotate_image(-10))
        self.rotate_left_button.pack(side=tk.LEFT)
        self.rotate_right_button = tk.Button(root, text="Rotate Right", command=lambda: self.rotate_image(10))
        self.rotate_right_button.pack(side=tk.LEFT)

        self.draw_box_button = tk.Button(root, text="Draw Box", command=self.set_draw_box)
        self.draw_box_button.pack(side=tk.LEFT)
        self.draw_circle_button = tk.Button(root, text="Draw Circle", command=self.set_draw_circle)
        self.draw_circle_button.pack(side=tk.LEFT)

        self.undo_button = tk.Button(root, text="Undo", command=self.undo_last_shape)
        self.undo_button.pack(side=tk.LEFT)

    def draw_grid(self):
        for i in range(0, canvas_width, meter_pixels):
            self.canvas.create_line(i, 0, i, canvas_width, fill="lightgray")
            self.canvas.create_line(0, i, canvas_height, i, fill="lightgray")

        self.canvas.create_line(canvas_width/2, 0, canvas_width/2, canvas_width, fill="black")
        self.canvas.create_line(0, canvas_height/2, canvas_height, canvas_height/2, fill="black")

    def load_image(self):
        file_path = "office.jpg"
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

    def set_draw_box(self):
        self.set_box = True
        self.set_circle = False

    def set_draw_circle(self):
        self.set_box = False
        self.set_circle = True

    def on_button_press(self, event):
        if self.set_box:
            if not self.is_drawing_box:
                self.is_drawing_line = True
                self.start_x = event.x
                self.start_y = event.y
                self.line = self.canvas.create_line(self.start_x, self.start_y, self.start_x, self.start_y, fill="black")
        elif self.set_circle:
            self.start_x = event.x
            self.start_y = event.y
            self.circle = self.canvas.create_oval(self.start_x, self.start_y, self.start_x, self.start_y, outline="black")

    def on_mouse_drag(self, event):
        if self.is_drawing_line:
            #only do vertical or horizontal lines
            if abs(event.x - self.start_x) > abs(event.y - self.start_y):
                end_x = event.x
                end_y = self.start_y
            else:
                end_x = self.start_x
                end_y = event.y
            self.canvas.coords(self.line, self.start_x, self.start_y, end_x, end_y)

            #self.canvas.coords(self.line, self.start_x, self.start_y, event.x, event.y)
        elif self.circle:
            diff_x = event.x - self.start_x
            diff_y = event.y - self.start_y
            max_diff = max(abs(diff_x), abs(diff_y))
            if diff_x > 0:
                diff_x = max_diff
            else:
                diff_x = -max_diff
            if diff_y > 0:
                diff_y = max_diff
            else:
                diff_y = -max_diff
            self.canvas.coords(self.circle, self.start_x-diff_x, self.start_y-diff_y, self.start_x+diff_x, self.start_y+diff_y)

    def on_button_release(self, event):
        if self.is_drawing_line:
            if abs(event.x - self.start_x) > abs(event.y - self.start_y):
                self.end_x = event.x
                self.end_y = self.start_y
            else:
                self.end_x = self.start_x
                self.end_y = event.y
            #self.end_x = event.x
            #self.end_y = event.y
            if (self.start_x, self.start_y) != (self.end_x, self.end_y):
                self.is_drawing_line = False
                self.is_drawing_box = True
        elif self.is_drawing_box:
                self.set_width_and_create_box(event.x, event.y)
                # self.set_box = False
                self.is_drawing_box = False
        elif self.circle:
            self.end_x = event.x
            self.end_y = event.y
            self.create_circle()

    def set_width_and_create_box(self, width_x, width_y):
        if self.line:
            width = abs(((self.end_x - self.start_x) * (width_y - self.start_y) - (self.end_y - self.start_y) * (width_x - self.start_x)) / ((self.end_x - self.start_x)**2 + (self.end_y - self.start_y)**2)**0.5)
            dx = self.end_x - self.start_x
            dy = self.end_y - self.start_y
            length = (dx**2 + dy**2)**0.5

            dx /= length
            dy /= length

            perp_dx = -dy * width / 2
            perp_dy = dx * width / 2

            if (width_x - self.start_x) * dy - (width_y - self.start_y) * dx < 0:
                perp_dx = -perp_dx
                perp_dy = -perp_dy

            x1 = self.start_x
            y1 = self.start_y
            x2 = self.end_x
            y2 = self.end_y
            x3 = self.end_x - perp_dx * 2
            y3 = self.end_y - perp_dy * 2
            x4 = self.start_x - perp_dx * 2
            y4 = self.start_y - perp_dy * 2

            self.rect = self.canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4, outline="black", fill="")
            self.shape_ids.append(self.rect)

            center_x = (x1 + x2 + x3 + x4) / 4
            center_y = (y1 + y2 + y3 + y4) / 4

            arena_x = (center_x - canvas_width/2) / meter_pixels
            arena_y = -(center_y - canvas_height/2) / meter_pixels

            box_id = f"box_{self.box_counter}"
            xml_size = f"{width/meter_pixels},{length/meter_pixels},0.5"
            xml_position = f"{arena_y},{-arena_x},0"

            angle = -math.degrees(math.atan2(dy, dx))
            xml_orientation = f"{angle},0,0"

            xml = f'''<box id="{box_id}" size="{xml_size}" movable="false">
    <body position="{xml_position}" orientation="{xml_orientation}"/>
</box>'''

            self.shapes.append(xml)
            print(xml)

            self.box_counter += 1

        self.lines.append(self.line)
        self.line = None
        # self.set_box = False

    def create_circle(self):
        if self.circle:
            radius = ((self.end_x - self.start_x)**2 + (self.end_y - self.start_y)**2)**0.5
            center_x = self.start_x 
            center_y = self.start_y

            arena_x = (center_x - canvas_width/2) / meter_pixels
            arena_y = -(center_y - canvas_height/2) / meter_pixels

            circle_id = f"circle_{self.circle_counter}"
            xml_radius = f"{radius/meter_pixels}"
            xml_position = f"{arena_y},{-arena_x},0"

            xml = f'''<cylinder id="{circle_id}" radius="{xml_radius}" height="0.5" temperature="0" movable="false">
    <body position="{xml_position}" orientation="0,0,0"/>
</cylinder>'''

            self.shapes.append(xml)
            self.shape_ids.append(self.circle)
            print(xml)

            self.circle_counter += 1

        self.circle = None
        # self.set_circle = False

    def undo_last_shape(self):
        if self.shapes and self.shape_ids:
            last_shape_id = self.shape_ids.pop()
            self.canvas.delete(last_shape_id)
            last_shape_xml = self.shapes.pop()
            if "<box" in last_shape_xml:
                self.box_counter -= 1
                line = self.lines.pop()
                self.canvas.delete(line)
            elif "<circle" in last_shape_xml:
                self.circle_counter -= 1
            


    def get_xml(self):
        return "\n".join(self.shapes)

    def zoom_in(self, event):
        self.scale_factor *= 1.1
        self.canvas.scale("all", event.x, event.y, 1.1, 1.1)
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self.display_image(self.scale_factor)

    def zoom_out(self, event):
        self.scale_factor *= 1/1.1
        self.canvas.scale("all", event.x, event.y, 1/1.1, 1/1.1)
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self.display_image(self.scale_factor)

def main():
    root = tk.Tk()
    app = DrawApp(root)
    root.mainloop()

    print("Final XML Output:\n")
    print(app.get_xml())

if __name__ == "__main__":
    main()