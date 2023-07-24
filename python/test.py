from utils import get_movements_16n

start_position = (0,0,0)

movements = get_movements_16n(start_position[0], start_position[1], start_position[2])
print("movements: ", movements)
print("len(movements): ", len(movements))