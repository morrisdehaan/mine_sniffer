"""
Graphic interface that relays status updates from the rover.
"""

import math, time
from typing import Optional, Tuple
import json
from dataclasses import dataclass
import tkinter as tk
import sensor.shared
import sensor.metal as metal
import shared
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from PIL import Image, ImageTk
import skimage

FRAME_BORDER_WIDTH = 1
WINDOW_COLUMNS, WINDOW_ROWS = 2, 4

# in pixels
SONAR_BLOCK_RADIUS = 150
SONAR_X_OFFSET, SONAR_Y_OFFSET = 100, -20
# in inches
PLOT_SIZE = (2.5, 2.5)

METAL_PLOT_SAMPLES = 50 # TODO?
# 
METAL_PLOT_WINDOW = 10

# display image height and width in pixels (ratio 3:4)
IR_DISPLAY_WIDTH, IR_DISPLAY_HEIGHT = 800, 600

# TODO: make members optional
@dataclass
class InterfaceUpdatePacket:
    # most recent sensor readings
    # metal detector reading + timestamp
    metal: Optional[list[Tuple[int, float]]]
    sonar: Optional[list[float]]
    ircam: Optional[list[float]]

    # positions rover has visited
    path: list[sensor.shared.Coord]
    # detected mines
    mines: list[sensor.shared.Coord]
    # predicted mines
    pmines: list[sensor.shared.Coord]

def parse_packet(bytes: bytes) -> Optional[InterfaceUpdatePacket]:
    if len(bytes) > 0:
        # read header
        # sensor data minus ir-camera
        sensor_size = int.from_bytes(bytes[:int(shared.ROVER_MSG_HEADER_SIZE/2)], "little")
        # ir-camera data
        ircam_size = int.from_bytes(bytes[int(shared.ROVER_MSG_HEADER_SIZE/2):shared.ROVER_MSG_HEADER_SIZE], "little")

        # read body
        # TODO: don't we want the other data?
        sensory = json.loads(bytes[shared.ROVER_MSG_HEADER_SIZE:shared.ROVER_MSG_HEADER_SIZE+sensor_size].decode("utf-8"))
        if ircam_size > 0:
            ircam = np.frombuffer(
                bytes[shared.ROVER_MSG_HEADER_SIZE+sensor_size:shared.ROVER_MSG_HEADER_SIZE+sensor_size+ircam_size],
                dtype="float16" # TODO: variable
            )
            np.reshape(ircam, newshape=(sensor.shared.IR_IMG_SIZE))
        else:
            ircam = None

        return InterfaceUpdatePacket(
            metal=sensory.get("metal", None),
            sonar=sensory.get("sonar", None),
            ircam=ircam,
            path=sensory.get("path", []),
            mines=sensory.get("mines", []),
            pmines=sensory.get("pmines", [])
        )
    else:
        return None

# TODO: mark private members
class Interface:
    def __init__(self):
        # value/timestamp samples are accumulated over time
        self.metal_samples = [[] for _ in range(metal.METAL_DETECTOR_COUNT)]
        self.metal_samples_times = [[] for _ in range(metal.METAL_DETECTOR_COUNT)]

        # create window
        self.window = tk.Tk("mine sniffer interface")
        # maximize screen
        self.window.state("zoomed")

        self.window.columnconfigure(list(range(WINDOW_COLUMNS)), weight=1, uniform="column")
        self.window.rowconfigure(list(range(WINDOW_ROWS)), weight=1, uniform="row")

        self.window.bind("<Key>", self.__process_key_press)
        
        self.metal_frame = tk.Frame(bg="salmon", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        # it's a canvas in order to draw laser lines
        self.sonar_frame = tk.Canvas(bg="white", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        self.ircam_frame = tk.Frame(bg="light blue", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        self.map_frame = tk.Frame(bg="black", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)

        self.metal_frame.grid(sticky="nsew")
        self.sonar_frame.grid(sticky="nsew")
        self.ircam_frame.grid(sticky="nsew")
        self.map_frame.grid(sticky="nsew")

        self.__set_default_frame_positions()

        # expand frame on click
        self.metal_frame.bind("<Button-1>", self.__expand_frame)
        self.sonar_frame.bind("<Button-1>", self.__expand_frame)
        self.ircam_frame.bind("<Button-1>", self.__expand_frame)
        self.map_frame.bind("<Button-1>", self.__expand_frame)

        self.__init_metal_frame()
        self.__init_sonar_frame()
        self.__init_ircam_frame()
        self.__init_map_frame()

        # for metal frame to be rendered correctly, it should happen at the end of initialization
        self.__resize_metal_frame(None)

        self.__manual_control_requested = False
        self.__request_control_switch = False

    """ Returns `True` if a control switch has been requested. """
    def update(self, data: Optional[InterfaceUpdatePacket]) -> bool:
        if data is not None:
            self.__update_metal_frame(data)
            self.__update_sonar_frame(data)
            self.__update_ircam_frame(data)
        
        self.window.update()

        control_switch_requested = self.__request_control_switch
        self.__request_control_switch = False
        return control_switch_requested

    def __init_metal_frame(self):
        PLOT_COLORS = ["red", "blue", "green"]

        self.metal_frame.bind("<Configure>", self.__resize_metal_frame)

        self.metal_frame.columnconfigure(list(range(metal.METAL_DETECTOR_COUNT)), weight=1)

        metal_title = tk.Label(master=self.metal_frame, text="metal detector")
        metal_title.pack()

        self.metal_plots = []
        for n in range(metal.METAL_DETECTOR_COUNT):
            # build plot
            fig, ax = plt.subplots()
            ax.set_xlim((-METAL_PLOT_WINDOW, 0))
            ax.set_yticklabels([])

            fig.set_facecolor("salmon")
            ax.set_facecolor((0.8, 0.8, 1.0, 0.3))
            ax.grid(color=(0.8, 0.8, 1.0))

            fig.set_size_inches(*PLOT_SIZE)
            # animated is set to true because blitting is exploited to improve render times
            (line,) = ax.plot([], [], animated=True)
            line.set_color(PLOT_COLORS[n])

            # render plot
            canvas = FigureCanvasTkAgg(fig, master=self.metal_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

            # the background cache will be set at the next configure event
            self.metal_plots.append((fig, ax, line, None))
        
    def __init_sonar_frame(self):
        # update lasers when frame is resized
        self.sonar_frame.bind("<Configure>", self.__resize_sonar_frame)

        sonar_title = tk.Label(master=self.sonar_frame, text="sonars")
        sonar_title.pack(pady=FRAME_BORDER_WIDTH + 2)

        # initialize sonars
        self.sonar_dists = [0.0] * sensor.shared.SONAR_COUNT
        self.sonar_labels = []
        self.sonar_lasers = []

        sonar_idx = 0
        for i in range(sensor.shared.SONAR_BLOCK_COUNT):
            angle = sensor.shared.SONAR_BLOCK_ANGLE / float(sensor.shared.SONAR_BLOCK_COUNT) * i
            x = math.cos(angle) * SONAR_BLOCK_RADIUS
            y = -math.sin(angle) * SONAR_BLOCK_RADIUS + SONAR_Y_OFFSET

            for x in [x + SONAR_X_OFFSET, -(x + SONAR_X_OFFSET)]:
                dist = self.sonar_dists[sonar_idx]

                # create distance label
                label = tk.StringVar(value=f"{dist:.0f}cm")
                tk.Label(master=self.sonar_frame, textvariable=label)\
                    .place(relx=0.5, rely=1.0, x=x, y=y, anchor=tk.CENTER)
                self.sonar_labels.append(label)
                
                # create laser. create_line does not support relative coordinates,
                #  so they need to be set at every configure event,
                #  which is first triggered when the window is displayed
                self.sonar_lasers.append(self.sonar_frame.create_line(0, 0, 0, 0, fill="red", width=2))

                sonar_idx += 1

    def __init_ircam_frame(self):
        ircam_title = tk.Label(master=self.ircam_frame, text="IR-camera")
        ircam_title.pack()

        self.img = ImageTk.PhotoImage(image=Image.fromarray(
            np.uint8(np.zeros((IR_DISPLAY_HEIGHT, IR_DISPLAY_WIDTH, 3))),
            mode="RGB"
        ))

        canvas = tk.Canvas(master=self.ircam_frame, width=IR_DISPLAY_WIDTH, height=IR_DISPLAY_HEIGHT)
        # TODO: use display size
        canvas.create_image(0, 0, anchor="nw", image=self.img)
        canvas.pack(pady=2)

    def __init_map_frame(self):
        # TODO: show route of rover + mine location + prediction mine locations
        map_title = tk.Label(master=self.map_frame, text="map")
        map_title.pack()

        self.__manual_control_btn_txt = tk.StringVar(value="Change to Manual Control")
        manual_control_btn = tk.Button(
            master=self.map_frame, textvariable=self.__manual_control_btn_txt, command=self.__change_control
        )
        manual_control_btn.pack()

    def __update_metal_frame(self, data: InterfaceUpdatePacket):
        # TODO: fix y-axis tick labels
        now = time.time()
        
        # repeat for each detector
        miny, maxy = 5000, -5000 # TODO
        for n in range(metal.METAL_DETECTOR_COUNT):
            samples, times = self.metal_samples[n], self.metal_samples_times[n]

            # add new data point
            if data.metal is not None:
                samples.append(data.metal[n][0])
                times.append(data.metal[n][1])

            # remove old data points
            for i, t in enumerate(times):
                if now - t > METAL_PLOT_WINDOW:
                    samples.pop(i)
                    times.pop(i)

            if len(samples) > 0:
                miny = min([miny, min(samples)])
                maxy = max([maxy, max(samples)])

        for n in range(metal.METAL_DETECTOR_COUNT):
            fig, ax, line, bg_cache = self.metal_plots[n]
            samples = self.metal_samples[n]

            # load new data points
            line.set_data([t - now for t in times], samples)
            # rescale axes
            ax.set_ylim(miny, maxy)
            ax.autoscale_view(scalex=False)
            
            # TODO: add blit comments
            fig.canvas.restore_region(bg_cache)

            ax.draw_artist(line)
            fig.canvas.blit(fig.bbox)
            fig.canvas.flush_events()

    def __resize_metal_frame(self, _: tk.Event):
        for n in range(metal.METAL_DETECTOR_COUNT):
            (fig, ax, line, _) = self.metal_plots[n]
            # draw background
            fig.canvas.draw()
            fig.canvas.flush_events()
            # cache background
            bg_cache = fig.canvas.copy_from_bbox(fig.bbox)
            self.metal_plots[n] = (fig, ax, line, bg_cache)

    def __update_sonar_frame(self, data: InterfaceUpdatePacket):
        MAPPING = [9, 4, 8, 3, 7, 2, 6, 1, 5, 0]

        if data.sonar is not None:
            for i in range(len(data.sonar)):
                dist = data.sonar[MAPPING[i]]
                self.sonar_dists[i] = dist
                self.sonar_labels[i].set(f"{dist:.0f}cm {i}")

            self.__resize_sonar_frame()

    def __resize_sonar_frame(self, _: Optional[tk.Event] = None):
        LENGTH_MULTIPLIER = 7.5
        MAX_DISPLAY_DIST = 10000

        framew, frameh = self.sonar_frame.winfo_width(), self.sonar_frame.winfo_height()

        sonar_idx = 0
        for i in range(sensor.shared.SONAR_BLOCK_COUNT):
            angle = sensor.shared.SONAR_BLOCK_ANGLE / float(sensor.shared.SONAR_BLOCK_COUNT) * i
            c, s = math.cos(angle), math.sin(angle)
            x = c * SONAR_BLOCK_RADIUS
            y = -s * SONAR_BLOCK_RADIUS + SONAR_Y_OFFSET

            # dir = direction of the laser, guaranteed to be a unit vector
            for dir, x in [((c, -s), x + SONAR_X_OFFSET), ((-c, -s), -(x + SONAR_X_OFFSET))]: 
                if self.sonar_dists[sonar_idx] == float("inf"):
                    dist = MAX_DISPLAY_DIST
                else:
                    dist = self.sonar_dists[sonar_idx]

                length = dist * LENGTH_MULTIPLIER
                # origin and end coordinates
                x0, y0 = x + framew/2, frameh+y
                x1, y1 = x0 + dir[0]*length, y0 + dir[1]*length
                
                # update laser position
                self.sonar_frame.coords(self.sonar_lasers[sonar_idx], x0, y0, x1, y1)
                sonar_idx += 1

    def __update_ircam_frame(self, data: InterfaceUpdatePacket):
        MIN_TEMP = 20.0
        MAX_TEMP = 45.0

        COLORS = [np.array([0,0,1]), np.array([0,1,1]), np.array([0,1,0]), np.array([1,1,0]), np.array([1,0,0])]

        if data.ircam is not None:          
            small_img = data.ircam.astype("float32")

            # map onto temperature range
            small_img = np.clip((small_img - MIN_TEMP) / (MAX_TEMP - MIN_TEMP), 0.0, 0.999)
            # we map each value to a color in COLORS
            col_idx = small_img * len(COLORS)
            # however, this is a fraction, so we mix the colors on either side
            col_idx0 = np.floor(col_idx)
            col_idx1 = col_idx0 + 1
            frac = col_idx - col_idx0

            # mix
            col0 = np.array([COLORS[int(i)] for i in col_idx0])
            col1 = np.array([COLORS[int(i)] for i in col_idx1])
            col_img = np.array([(col1 - col0) * frac + col0 for col0, col1, frac in zip(col0, col1, frac)])\
                .reshape((sensor.shared.IR_IMG_HEIGHT, sensor.shared.IR_IMG_WIDTH, 3))

            # upscale image
            interpolated_img = skimage.transform.resize(col_img, (IR_DISPLAY_HEIGHT, IR_DISPLAY_WIDTH))

            self.img.paste(Image.fromarray(np.uint8(interpolated_img * 255), mode="RGB"))

    def __change_control(self):
        self.__manual_control_requested = not self.__manual_control_requested
        self.__request_control_switch = True
        
        if self.__manual_control_requested:
            self.__manual_control_btn_txt.set("Change to Automatic Control")
        else:
            self.__manual_control_btn_txt.set("Change to Manual Control")

    # TODO: rename
    def __set_default_frame_positions(self):
        self.metal_frame.grid(column=0, row=0, columnspan=1, rowspan=1)
        self.sonar_frame.grid(column=0, row=1, columnspan=1, rowspan=1)
        self.ircam_frame.grid(column=0, row=2, columnspan=1, rowspan=2)
        self.map_frame.grid(column=1, row=0, columnspan=1, rowspan=4)

    def __expand_frame(self, event: tk.Event):
        # hide all frames
        self.metal_frame.grid_remove()
        self.sonar_frame.grid_remove()
        self.ircam_frame.grid_remove()
        self.map_frame.grid_remove()

        # expand the selected frame
        event.widget.grid(column=0, row=0, columnspan=WINDOW_COLUMNS, rowspan=WINDOW_ROWS)

    def __process_key_press(self, event: tk.Event):
        if event.keysym == "Escape":
            self.__set_default_frame_positions()

    def is_window_alive(self) -> bool:
        try:
            # returns an exception if the window has been closed
            return self.window.winfo_exists()
        except: # TODO: set exception type?
            return False
        
    def manual_control_requested(self) -> bool:
        return self.__manual_control_requested

# TODO: remove
if __name__ == "__main__":
    interface = Interface()

    # TODO: weg
    last_frame_time = last_data_update_time = time.time()

    # TODO: manage some fps?
    while interface.is_window_alive():
        # TODO: accept sensor data over TCP
        now = time.time()

        delta = now - last_frame_time
        if delta != 0:
            print(f"fps: {1.0 / delta}")
        last_frame_time = now

        if now - last_data_update_time > 0.1:
            sensor_data = InterfaceUpdatePacket(
                metal=[(int(math.cos(now * 5) * 100), now)] * metal.METAL_DETECTOR_COUNT,
                sonar=[50.0 + math.sin(now) * 20] * sensor.shared.SONAR_COUNT,
                ircam=None, path=None, mines=None, pmines=None
            )
            last_data_update_time = now
        else:
            sensor_data = None

        interface.update(sensor_data)
