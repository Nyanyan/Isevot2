import subprocess
from othello import *

cmd = 'Egaroucid_for_Console/Egaroucid_for_Console_7_6_0_SIMD.exe -quiet -l 21 -noise'

egaroucid = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=None)

def cleanup_ai():
    egaroucid.stdin.write('quit\n'.encode('utf-8'))
    egaroucid.stdin.flush()

def ai_get_best_move(othello):
    s = othello.get_board_str()
    egaroucid.stdin.write(f"setboard {s}\n".encode())
    egaroucid.stdin.flush()
    egaroucid.stdin.write(f"go\n".encode())
    egaroucid.stdin.flush()
    line = ''
    while line == '' or line == '>':
        line = egaroucid.stdout.readline().decode().replace('\r', '').replace('\n', '')
    coord = line[-2:].lower()
    try:
        x = ord(coord[0]) - ord('a')
        y = int(coord[1]) - 1
    except:
        print('error')
        print(s)
        print(coord)
        exit()
    return x, y