import serial
import sys
import signal
import time
from othello import *
from camera import *
from ai import *

def start_serial_communication(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Serial communication started on {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error starting serial communication: {e}")
        return None

#port = "COM5"  # Replace with your serial port
port = "COM8"
baudrate = 9600  # Replace with your desired baud rate
serial_connection = start_serial_communication(port, baudrate)

def wait_finish():
    while serial_connection.read(1) != b'0':
        pass


def cleanup_serial():
    if serial_connection:
        serial_connection.close()
        print("Serial communication closed.")

def cleanup():
    cleanup_camera()
    cleanup_serial()
    cleanup_ai()
    print('bye!')

def sig_handler(signum, frame) -> None:
    sys.exit(1)

def format_digit(x):
    res = '+' if x > 0 else '-'
    res += str(abs(x) // 10)
    res += str(abs(x) % 10)
    return res

def main():
    signal.signal(signal.SIGTERM, sig_handler)
    try:
        #serial_connection.write('i'.encode())
        #wait_finish()
        #serial_connection.write('h'.encode())
        #wait_finish()

        # check which plays black
        if serial_connection:
            serial_connection.write('t'.encode())
            received = serial_connection.read(1)  # Read one byte
            if received == b'r':
                AI_PLAYER = BLACK
            else:
                AI_PLAYER = WHITE
            black_player = 'Robot' if AI_PLAYER == BLACK else 'Human'
            white_player = 'Robot' if AI_PLAYER == WHITE else 'Human'
            print('Black: ' + black_player + ' White: ' + white_player)

        robot_player_str = 'b' if AI_PLAYER == BLACK else 'w'
        human_player_str = 'w' if AI_PLAYER == BLACK else 'b'

        start_game_cmd = 'g' + robot_player_str
        print(start_game_cmd)
        serial_connection.write(start_game_cmd.encode())
        wait_finish()

        while True:
            if serial_connection:

                # check turn
                need_to_put_disc = False
                serial_connection.write('b'.encode())
                received = serial_connection.read(1)  # Read one byte
                if received == b'y':
                    print("robot's turn")
                    need_to_put_disc = True
                    serial_connection.write('sr'.encode())
                    wait_finish()

                
                if need_to_put_disc:
                    serial_connection.write(b'c') # ready for camera
                    wait_finish()
                    board_arr = None
                    disc_slip_mm = None
                    while board_arr == None:
                        board_arr, disc_slip_mm = get_board()
                    othello = Othello(board_arr, AI_PLAYER)
                    othello.print()
                    for y in range(HW):
                        for x in range(HW):
                            slip_r = max(disc_slip_mm[y][x][0], disc_slip_mm[y][x][1])
                            #slip_r = math.sqrt(disc_slip_mm[y][x][0] ** 2 + disc_slip_mm[y][x][1] ** 2)
                            if slip_r > 8:
                                color_str = 'b' if othello.grid[y][x] == BLACK else 'w'
                                modify_cmd = 'm' + color_str + str(HW - 1 - x) + str(HW - 1 - y) + format_digit(disc_slip_mm[y][x][0]) + format_digit(disc_slip_mm[y][x][1])
                                print(modify_cmd)
                                serial_connection.write(modify_cmd.encode())
                                wait_finish()
                    if othello.get_legal():
                        print("AI's turn")
                        selected_move_x, selected_move_y = ai_get_best_move(othello)
                        print('selected', selected_move_x, selected_move_y)
                    else:
                        print("No legal moves available.")
                        continue
                    flipped = othello.get_flipped(selected_move_y, selected_move_x)
                    put_cmd = 'p' + robot_player_str + str(HW - 1 - selected_move_x) + str(HW - 1 - selected_move_y)
                    print(put_cmd)
                    serial_connection.write(put_cmd.encode())
                    wait_finish()
                    dy = [-1, -1, -1,  0,  1,  1,  1,  0]
                    dx = [-1,  0,  1,  1,  1,  0, -1, -1]
                    for dr in range(8):
                        y = selected_move_y
                        x = selected_move_x
                        for i in range(1, 8):
                            y += dy[dr]
                            x += dx[dr]
                            if y < 0 or y >= HW or x < 0 or x >= HW:
                                break
                            if flipped[y][x]:
                                print('flipped', y, x)
                                flip_cmd = 'f' + human_player_str + str(HW - 1 - x) + str(HW - 1 - y)
                                print(flip_cmd)
                                serial_connection.write(flip_cmd.encode())
                                wait_finish()
                    serial_connection.write('q'.encode()) # push clock
                    wait_finish()
                    serial_connection.write('sh'.encode()) # human's turn
                    wait_finish()
                    serial_connection.write('h'.encode()) # set arm to homw
                    wait_finish()
            #time.sleep(0.1)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
            

if __name__ == "__main__":
    sys.exit(main())
