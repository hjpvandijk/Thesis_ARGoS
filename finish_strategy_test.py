import pygame
import math

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
AGENT_RADIUS = 10
GOAL_RADIUS = 1.0 * 100  # Convert meters to pixels (assuming 100px = 1m)
GOAL_POS = (WIDTH // 2, HEIGHT // 2)
FPS = 16
SHRINK_RATE = 0.01 * 100  # Convert to pixels
GROW_RATE = 0.002 * 100  # Convert to pixels

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Setup display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Movable Agent with Goal")
clock = pygame.time.Clock()

# Agent position
agent_x, agent_y = WIDTH // 4, HEIGHT // 4
# min_distance = math.dist((agent_x, agent_y), GOAL_POS)
min_distance = 9999999

done = False
while not done:
    screen.fill(WHITE)
    
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    # Get mouse position (agent moves with mouse)
    agent_x, agent_y = pygame.mouse.get_pos()
    
    # Calculate distance to goal
    current_distance = math.dist((agent_x, agent_y), GOAL_POS)
    
    # Adjust goal radius
    if current_distance < min_distance:
        GOAL_RADIUS = max(5, GOAL_RADIUS - SHRINK_RATE)  # Minimum size of 5px
    else:
        GOAL_RADIUS = min(9999, GOAL_RADIUS + GROW_RATE)  # Maximum size of 200px
    
    #Write goal radius in the center of the 'goal'
    font = pygame.font.Font(None, 36)
    text = font.render(f'{GOAL_RADIUS}', True, RED)
    text_rect = text.get_rect(center=(WIDTH // 2, HEIGHT // 2))
    screen.blit(text, text_rect)


    min_distance = min(current_distance, min_distance)

    #If agent is inside the goal, make the goal green
    if current_distance < GOAL_RADIUS:
        pygame.draw.circle(screen, (0, 255, 0), GOAL_POS, int(GOAL_RADIUS), 2)
    else:
        pygame.draw.circle(screen, RED, GOAL_POS, int(GOAL_RADIUS), 2)
    
    # # Draw goal circle
    # pygame.draw.circle(screen, RED, GOAL_POS, int(GOAL_RADIUS), 2)
    
    # Draw agent
    pygame.draw.circle(screen, BLUE, (agent_x, agent_y), AGENT_RADIUS)
    
    # Refresh screen
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
