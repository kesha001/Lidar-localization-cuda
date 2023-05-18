import math
import pygame
import pandas as pd

# Pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
# Robot setup
player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)
player_angle = 0


data_set = []

# Wall setup
walls = [
    (pygame.Vector2(10, 10),    pygame.Vector2(1270, 10)),
    (pygame.Vector2(10, 10),    pygame.Vector2(10, 710)),
    (pygame.Vector2(10, 710),   pygame.Vector2(1270, 710)),
    (pygame.Vector2(1270, 10),  pygame.Vector2(1270, 710)),
    # (pygame.Vector2(10, 100),    pygame.Vector2(200, 100)),
    # (pygame.Vector2(10, 250),    pygame.Vector2(400, 500)),
    # right angle
    (pygame.Vector2(900, 10),    pygame.Vector2(700, 300)),
    (pygame.Vector2(700, 300),    pygame.Vector2(950, 300)),
    # right door hole
    (pygame.Vector2(950, 300),    pygame.Vector2(950, 425)),
    (pygame.Vector2(950, 575),    pygame.Vector2(950, 710)),
    # left angle
    (pygame.Vector2(300, 10),    pygame.Vector2(450, 300)),
    (pygame.Vector2(450, 300),    pygame.Vector2(200, 300)),
    # left door hole
    (pygame.Vector2(200, 300),    pygame.Vector2(200, 425)),
    (pygame.Vector2(200, 575),    pygame.Vector2(200, 710)),
    # (pygame.Vector2(850, 300),    pygame.Vector2(850, 450)), 
    # square
    (pygame.Vector2(500, 500),    pygame.Vector2(700, 500)),
    (pygame.Vector2(700, 500),    pygame.Vector2(700, 700)),
    (pygame.Vector2(700, 700),    pygame.Vector2(500, 700)),
    (pygame.Vector2(500, 700),    pygame.Vector2(500, 500)),

    (pygame.Vector2(600, 10),     pygame.Vector2(600, 300)),
    
]
# Lidar parameters
lidar_range = 999
lidar_max_angle = math.pi#math.pi/4
lidar_step = 4*math.pi/180#math.pi/180
lidar_num_steps = 2*180//4
lidar_distance_error = 0.5

# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    def ccw(A, B, C):
        return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

def intersection_point(A, B, C, D):
    t = ((A.x - C.x)*(C.y - D.y) - (A.y - C.y)*(C.x - D.x)) / ((A.x - B.x)*(C.y - D.y) - (A.y - B.y)*(C.x - D.x))
    return pygame.Vector2(A.x + t*(B.x - A.x), A.y + t*(B.y - A.y)), t

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill("black")
    # Draw the walls (lines)
    for wall in walls:
        pygame.draw.line(screen, "white", wall[0], wall[1])
    # Draw the robot (triangle)
    pygame.draw.polygon(screen, "red", [[player_pos.x + math.cos(player_angle) * 10, player_pos.y + math.sin(player_angle) * 10],
                                        [player_pos.x - math.cos(player_angle) * 10 - math.sin(player_angle) * 5, player_pos.y - math.sin(player_angle) * 10 + math.cos(player_angle) * 5],
                                        [player_pos.x - math.cos(player_angle) * 10 + math.sin(player_angle) * 5, player_pos.y - math.sin(player_angle) * 10 - math.cos(player_angle) * 5]])
    # Movement
    keys = pygame.key.get_pressed()
    moved = False
    if keys[pygame.K_w]:
        new_pos = player_pos.copy()
        new_pos.x += math.cos(player_angle) * 300 * dt
        new_pos.y += math.sin(player_angle) * 300 * dt
        # Collision detection
        collision = False
        for wall in walls:
            if intersect(player_pos, new_pos, wall[0], wall[1]):
                collision = True
        if not collision:
            player_pos = new_pos
            moved = True
    if keys[pygame.K_s]:
        new_pos = player_pos.copy()
        new_pos.x -= math.cos(player_angle) * 300 * dt
        new_pos.y -= math.sin(player_angle) * 300 * dt
        # Collision detection
        collision = False
        for wall in walls:
            if intersect(player_pos, new_pos, wall[0], wall[1]):
                collision = True
        if not collision:
            player_pos = new_pos
            moved = True
    if keys[pygame.K_a]:
        player_angle -= 0.1
        moved = True
    if keys[pygame.K_d]:
        player_angle += 0.1
        moved = True

    cur_frame_data = [player_pos.x, player_pos.y, player_angle,]
    # Lidar scan
    angle = player_angle - lidar_max_angle
    for i in range(lidar_num_steps):
        max_lidar_point = pygame.Vector2(player_pos.x + math.cos(angle) * lidar_range, player_pos.y + math.sin(angle) * lidar_range)
        pygame.draw.line(screen, "green", max_lidar_point, player_pos, 1)
        min_dist, intersect_point = 99999, None
        closest_intersect_point = None
        for wall in walls:
            if intersect(player_pos, max_lidar_point, wall[0], wall[1]):
                (intersect_point, dist) = intersection_point(player_pos, max_lidar_point, wall[0], wall[1])
                if dist < min_dist:
                    min_dist = dist
                    closest_intersect_point = intersect_point
        if min_dist <= lidar_range:
            pygame.draw.circle(screen, "red", closest_intersect_point, 5)
        if moved:
            print(f'X: {round(player_pos.x, 2)}, Y: {round(player_pos.y, 2)}, Theta: {round(player_angle, 2)}, Lidar: {round(angle, 2)}, Dist: {round(min_dist, 2)}')
            cur_frame_data.append(min_dist)
        angle += lidar_step
    
    pygame.display.flip()
    if len(cur_frame_data)>3:
        data_set.append(cur_frame_data)
    dt = clock.tick(60) / 1000

pygame.quit()

# write into dataframe
column_names = {0:"x", 1:"y", 2:"player_angle"}
df = pd.DataFrame(data_set)
for i in range(3, df.shape[1]):
    column_names.update({i: f"distance_{i-3}"})
df = df.rename(columns=column_names)

df.to_csv("lidar_data_new.csv")
