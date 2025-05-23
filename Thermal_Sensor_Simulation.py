import random
import time
from time import sleep
import math
import tkinter as tk

#Displays a grid that you give it in printed form
def show_grid(Grid):
    for i in range(len(Grid)):  
        row_str = ""
        for j in range(len(Grid[i])):  
            row_str += str(Grid[i][j]) 
            row_str += " "
        print(row_str)

#This will take in a point on an array, and populate a octigon of 1s around it
def create_Heat_Octagon(grid, x, y):
    grid[x-1][y-3] = 1
    grid[x][y-3] = 1
    grid[x+1][y-3] = 1

    grid[x-2][y-2] = 1
    grid[x+2][y-2] = 1

    grid[x-3][y-1] = 1
    grid[x+3][y-1] = 1
    grid[x-3][y] = 1
    grid[x+3][y] = 1
    grid[x-3][y+1] = 1
    grid[x+3][y+1] = 1

    grid[x-2][y+2] = 1
    grid[x+2][y+2] = 1

    grid[x-1][y+3] = 1
    grid[x][y+3] = 1
    grid[x+1][y+3] = 1

    grid[x-2][y-4] = 1
    grid[x-1][y-4] = 1
    grid[x][y-4] = 1
    grid[x+1][y-4] = 1
    grid[x+2][y-4] = 1

    grid[x-3][y-3] = 1

    grid[x-2][y-3] = 1 
    grid[x-2][y+3] = 1 
    grid[x-3][y-2] = 1 
    grid[x-3][y+2] = 1 
    grid[x+2][y-3] = 1  
    grid[x+2][y+3] = 1  
    grid[x+3][y-2] = 1  
    grid[x+3][y+2] = 1 

    grid[x+3][y-3] = 1

    grid[x-4][y-2] = 1
    grid[x+4][y-2] = 1

    grid[x-4][y-1] = 1
    grid[x+4][y-1] = 1
    grid[x-4][y] = 1
    grid[x+4][y] = 1
    grid[x-4][y+1] = 1
    grid[x+4][y+1] = 1

    grid[x-4][y+2] = 1
    grid[x+4][y+2] = 1

    grid[x-3][y+3] = 1
    grid[x+3][y+3] = 1

    grid[x-2][y+4] = 1
    grid[x-1][y+4] = 1
    grid[x][y+4] = 1
    grid[x+1][y+4] = 1
    grid[x+2][y+4] = 1

#This will take the camera position, and populate a grid with c's to show where the camera is looking
def Show_Cam_On_Map(grid, cam_x, cam_y, cam_width, cam_length): 

    for i in range(cam_width):
        for j in range(cam_length):
            Pos_x = cam_x - (cam_width // 2) + i
            Pos_y = cam_y - (cam_length // 2) + j
            if 0 <= Pos_x < len(grid[0]) and 0 <= Pos_y < len(grid):
                if grid[Pos_y][Pos_x] != 1:
                    grid[Pos_y][Pos_x] = "c"

#This will clean up the "c" and 2s and 3s from the grid, so that it can be used again
def Clean_Map(grid):
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == "c" or grid[i][j] == 2 or grid[i][j] == 3:
                grid[i][j] = 0

#Will take the camera position, and will grab the heat values that it can see from the heat map
def camera(Width, Length, x, y, grid):
    camera_view = [[0 for _ in range(Width)] for _ in range(Length)]  

    for i in range(Width):
        for j in range(Length):
            Pos_x = x - (Width // 2) + i
            Pos_y = y - (Length // 2) + j
            if 0 <= Pos_x < len(grid) and 0 <= Pos_y < len(grid[0]):
                camera_view[j][i] = grid[Pos_y][Pos_x]
            else:
                camera_view[j][i] = 0 

    return camera_view

#Creates the heat map, where the non fire values are between 0 and 40, and the fire values are between 55 and 100
def create_heat_map(grid):
    heat_map = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]  
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == 1:
                heat_map[i][j] = random.randint(55, 100)
            else:
                heat_map[i][j] = random.randint(0, 40)
    return heat_map

#This is the main function that determines how the thermal camera detects heat. This will take the camera heat data,
#Then it will set a cuttoff of 50, and if the heat value is above that, it will count it as a heat source.
#It will then average the heat sources, and return the average x and y position of the heat source, as well as the distance and angle to it.
#It only does this however if there are more than 10 heat sources detected.
def detect_heat(heat_map, x, y):
    count = 0
    x_s = 0
    y_s = 0
    for i in range(len(heat_map)):
        for j in range(len(heat_map[i])):
            
            if heat_map[i][j] >50:
                heat_map[i][j] = 1
                count += 1
                x_s += x - len(heat_map[0]) // 2 + j
                y_s += y - len(heat_map) // 2 + i
            else:
                heat_map[i][j] = 0
    if count > 10:
        x_avg = x_s // count
        y_avg = y_s // count
        dist = ((x_avg - x)**2 + (y_avg - y)**2)**0.5
        d_psi = math.atan2(x_avg - x,y_avg - y) * 180 / math.pi
        return (x_avg, y_avg,dist,d_psi )
    else:
        return (-1, -1, -1,-1)

#updates the map display
def update_map_display(grid, map_window):
    map_text = "\n".join([" ".join(map(str, row)) for row in grid])
    map_label.config(text=map_text)
    map_window.update()

#updates the camera view display
def update_camera_display(camera_view, camera_window):
    camera_text = "\n".join([" ".join(map(str, row)) for row in camera_view])
    camera_label.config(text=camera_text)
    camera_window.update()

#Draws the god map
def draw_grid(canvas, grid):
    canvas.delete("all")  #Clears the canvas
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            x1 = j * cell_size
            y1 = i * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size

            #1 is the fire, 0 is empty space and c is the camera
            
            if grid[i][j] == 1:
                color = "red"
            elif grid[i][j] == "c": 
                color = "green"
            elif grid[i][j] == 2:  
                color = "gold"
            else: 
                color = "white"

            #Draws the rectangle
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")

#This function draws an arrow on the canvas
def draw_arrow(canvas, start_x, start_y, end_x, end_y):
    canvas.create_line(
        start_x * cell_size + cell_size // 2,
        start_y * cell_size + cell_size // 2,
        end_x * cell_size + cell_size // 2,
        end_y * cell_size + cell_size // 2,
        arrow=tk.LAST, fill="blue", width=4, tags="arrow"
    )

#This function will draw the filtered camera view on the canvas
def draw_camera_view(canvas, camera_view):
    canvas.delete("all")  #Clear the canvas
    for i in range(len(camera_view)):
        for j in range(len(camera_view[i])):
            x1 = j * cell_size
            y1 = i * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size

            #1 is the fire, 0 is empty space
            if camera_view[i][j] == 1: 
                color = "red"
            else: 
                color = "white"

            #Draws the rectangle
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")


#This function will take the current position of the camera, and the waypoints, and calculate the velocity vector to the next waypoint.
#It will also check if the camera is at the waypoint, and if it is, it will move to the next waypoint.
def Calc_Velocity_Waypoints(x,y,Waypoints,Current_Waypoint):
    Max_Speed = 3
    if Current_Waypoint >= len(Waypoints):
        print("All waypoints reached.")
        return 0, 0

    target_x, target_y = Waypoints[Current_Waypoint]
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx**2 + dy**2)

    if distance == 0:
        print("Reached waypoint:", Current_Waypoint)
        Current_Waypoint += 1
        return 0, 0, Current_Waypoint
    else:
        velocity_x = dx
        velocity_y = dy

        Vel_Magnitude = math.sqrt(velocity_x**2 + velocity_y**2)

        if Vel_Magnitude > Max_Speed:
            velocity_x *= Max_Speed / Vel_Magnitude
            velocity_y *= Max_Speed / Vel_Magnitude
        print("Current Waypoint:", Current_Waypoint, "Velocity:", velocity_x, velocity_y)
        return velocity_x, velocity_y, Current_Waypoint
    
#This function will take the waypoints and the width and height of the grid, and correct the waypoints to fit within the grid.
#It also turns the waypoint which are in ft to pixels, so that they can be used in the grid.
def correct_Waypoints(Waypoints, Width, Height,cell_per_ft):
    for i in range(len(Waypoints)):
        Waypoints[i] = (int(Waypoints[i][0] * cell_per_ft+ Width//2), int(Waypoints[i][1] * cell_per_ft))
    return Waypoints
    
#This function will take the fire position and the camera position, and calculate the velocity vector towards the fire.
#It will also check if the camera is at the fire, and if it is, it will stop moving.
def Move_to_Fire(Fx,Fy,Cam_x,Cam_y):
    Max_Speed = 3
    dx = Fx - Cam_x
    dy = Fy - Cam_y
    distance = math.sqrt(dx**2 + dy**2)

    if distance == 0:
        return 0, 0, True
    else:
        velocity_x = dx
        velocity_y = dy

        Vel_Magnitude = math.sqrt(velocity_x**2 + velocity_y**2)

        if Vel_Magnitude > Max_Speed:
            velocity_x *= Max_Speed / Vel_Magnitude
            velocity_y *= Max_Speed / Vel_Magnitude

        return velocity_x, velocity_y, False
    
#This function will draw the heat map, and will do a white hot color scheme for the heat map.
def draw_heatmap(canvas, heat_map):
    canvas.delete("all")  
    for i in range(len(heat_map)):
        for j in range(len(heat_map[i])):
            x1 = j * cell_size
            y1 = i * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size

            
            value = heat_map[i][j]
            red = min(255, int((value) * 5))
            green = min(255, int((value) * 5)) 
            blue = min(255, int((value) * 5))

            
            color = f"#{red:02x}{green:02x}{blue:02x}"

            
            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="black")

#This will draw the raw camera heat values, and will do a white hot color scheme for the camera view.
def draw_raw_camera_view(canvas, camera_view):
    canvas.delete("all")  
    for i in range(len(camera_view)):
        for j in range(len(camera_view[i])):
            x1 = j * cell_size
            y1 = i * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size
            value = camera_view[i][j]
            red = min(255, int((value) * 5))
            green = min(255, int((value) * 5)) 
            blue = min(255, int((value) * 5))
            color = f"#{red:02x}{green:02x}{blue:02x}"
            
            canvas.create_rectangle(x1, y1, x2, y2, fill=color)















#Sets the simulation to run
run = True

count_Wait = 0


#These are the waypoints in ft that the drone will go to
Way_Points = {
    0: (0, 0),
    1: (0, 5),
    2: (5, 5),
    3: (5, 34),
    4: (-5, 34),
    5: (-5, 9),
    6: (0, 9),
    7: (0, 30),
}

#Number of cells that corispond to a ft
Cells_Per_Foot = 3

#Calculates the width and length of the map
Width = 20*Cells_Per_Foot
Height = 38*Cells_Per_Foot

#Puts the fire in the middle
Fire_y = 16*Cells_Per_Foot
Fire_x = 10*Cells_Per_Foot

#Corrects the waypoints so they arein grid values
Way_Points = correct_Waypoints(Way_Points, Width, Height,Cells_Per_Foot)

#Creates the god map
grid = [[0 for _ in range(Width)] for _ in range(Height)]  

#Adds the fire to the god map
create_Heat_Octagon(grid, Fire_y, Fire_x)

#Creates the heat map based on the god map
heat_map = create_heat_map(grid)


#Sets the camera position to the first waypoint
Cam_x = Way_Points[0][0]
Cam_y = Way_Points[0][1]

#Sets camera width and length
camera_width = 30
camera_length = 20

#Initialize camera view
camera_view = [[0 for _ in range(camera_width)] for _ in range(camera_length)]

#Create god map window
map_window = tk.Tk()
map_window.title("Map")
map_label = tk.Label(map_window, text="", font=("Courier", 10), justify="left")
map_label.pack()

#Gets the screen dimensions
screen_width = map_window.winfo_screenwidth()
screen_height = map_window.winfo_screenheight()

#Calculates cell size to fit the grid within the screen
cell_size = min(screen_width // Width, screen_height // Height)

#Add a canvas to the map window for drawing the arrow
map_canvas = tk.Canvas(map_window, width=Width * cell_size, height=Height * cell_size, bg="white")
map_canvas.pack()

#Creates camera view window
camera_window = tk.Tk()
camera_window.title("Camera View")
camera_label = tk.Label(camera_window, text="", font=("Courier", 10), justify="left")
camera_label.pack()

#Adds a canvas to the camera view window
camera_canvas = tk.Canvas(camera_window, width=camera_width * cell_size, height=camera_length * cell_size, bg="white")
camera_canvas.pack()

#Creates raw heatmap window
heatmap_window = tk.Tk()
heatmap_window.title("Raw Heatmap")
heatmap_canvas = tk.Canvas(heatmap_window, width=Width * cell_size, height=Height * cell_size, bg="white")
heatmap_canvas.pack()

#Creates raw camera view window
raw_camera_window = tk.Tk()
raw_camera_window.title("Raw Camera View")
raw_camera_canvas = tk.Canvas(raw_camera_window, width=camera_width * cell_size, height=camera_length * cell_size, bg="white")
raw_camera_canvas.pack()


#initilized variables
Current_Waypoint = 0
Heat_Detected = False
At_Fire = False
global count_IDK
count_IDK = 0

#MAIN FUNCTION
def update_simulation():

    #Sets global variables that are needed every time
    global Cam_x, Cam_y, run, Current_Waypoint, Heat_Detected, At_Fire, x_s, y_s,count_IDK

    #Stops the smulation
    if not run:
        return  

    #First checks to see if any heat was detected from the camera
    if Heat_Detected:

        #If we are at the fire, stop moving
        if At_Fire:
            print("At Fire")
            Cam_Velocity_x = 0
            Cam_Velocity_y = 0

        #If we are not at the fire, move to the fire
        else:
            print("Moving to Fire")
            Cam_Velocity_x, Cam_Velocity_y, At_Fire = Move_to_Fire(x_s, y_s, Cam_x, Cam_y)

    #If no heat was detected, the drone should continue to the next waypoint
    else:
        [Cam_Velocity_x, Cam_Velocity_y, Current_Waypoint] = Calc_Velocity_Waypoints(Cam_x, Cam_y, Way_Points, Current_Waypoint)

    #Moves the camera based off of the velocity
    if count_IDK > 1:
        Cam_x += int(Cam_Velocity_x)
        Cam_y += int(Cam_Velocity_y)
    count_IDK += 1
    

    #Ensures the camera stays within bounds
    if Cam_x < 0:
        Cam_x = 0
    if Cam_x >= Width:
        Cam_x = Width - 1
    if Cam_y < 0:
        Cam_y = 0
    if Cam_y >= Height:
        Cam_y = Height - 1

    #gets the heat values the camera can see from its position
    camera_view_raw = camera(camera_width, camera_length, Cam_x, Cam_y, heat_map)

    #Draws the raw camera view
    draw_raw_camera_view(raw_camera_canvas, camera_view_raw)

    #Plots the camera on the god map
    Show_Cam_On_Map(grid, Cam_x, Cam_y, camera_width, camera_length)

    #Runs the detection algorithm and will return the x and y position of the heat source, as well as the distance and angle to it.
    [x_s, y_s, dist, angle] = detect_heat(camera_view_raw, Cam_x, Cam_y)

    #Draws the god map
    draw_grid(map_canvas, grid)

    #Draws the filtered camera view
    draw_camera_view(camera_canvas, camera_view_raw) 

    #Draws the heatmap
    draw_heatmap(heatmap_canvas, heat_map)  
    
    #If there was a valid heat source detected, draw the arrow on the map and camera view
    if x_s != -1 and y_s != -1 and dist != -1 and angle != -1:
        print("Heat detected at: ", x_s, y_s)
        Heat_Detected = True
        draw_arrow(map_canvas, Cam_x, Cam_y, x_s, y_s)
        draw_arrow(camera_canvas, camera_width // 2, camera_length // 2,
                   camera_width // 2 - (Cam_x - x_s), camera_length // 2 - (Cam_y - y_s))
    #If there wasn't a valid heat source detected, set the heat detected to false and print that no heat was detected
    else:
        Heat_Detected = False
        print("No heat detected")

    #Clean the god map for the next iteration
    Clean_Map(grid)

    #Schedule the next update after 1 second (1000 milliseconds)
    if count_IDK > 1:
        map_window.after(1000, update_simulation)
    else:
        map_window.after(100, update_simulation)

#Start the simulation
update_simulation()

#Run the main event loop for the GUI
map_window.mainloop()
