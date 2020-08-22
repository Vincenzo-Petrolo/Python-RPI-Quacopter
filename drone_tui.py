import curses
import time
import threading

from quadcopter import *
def main():
    
    '''
        Standard methods call to ncurses interface
    '''
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(1)

    '''
        Prints the title of the TUI
    '''
    dims = stdscr.getmaxyx()
    stdscr.addstr(1,int(dims[1]/2),"DRONE TUI")

    '''
        Data structures, quadcopter
    '''
    quadricottero = quadcopter(4,27,17,22,False)

    '''
        Now we'll start a thread that will run until the quadricottero.stop() method
        is called. That thread will represent the quadcopter propellers being powered
        as long as that thread runs, the quadcopter is flying
    '''
    thread = threading.Thread(target=quadcopter.start,args=(quadricottero,))
    try:
        thread.start()
    except:
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        print("Erorr can't start quadcopter")
        return
    '''
        Main loop : Here the values of the motors will be printed and we'll be able
        to change the values of PID for PITCH and ROLL respectively. (Useful feature
        for testing the correcteness of PID parameteres)
    '''
    c = 0x0
    i = 0
    PID = ["P : ","I : ","D :"]
    while c != ord('q'):
        stdscr.addstr(2,5,"ROLL")
        stdscr.addstr(5,5,"PITCH")
        for string in PID: 
            KP = quadricottero.get_KP[i]
            KR = quadricottero.get_KR[i]
            stdscr.addstr(3 + i,5,string + KP)
            stdscr.addstr(6 + i,5,string + KR)
            i += 1
            
        c = stdscr.getch()
        stdscr.refresh()
    
    '''
        Stops the quadcopter
    '''
    quadricottero.stop()
    '''
        End of the curses mode
    '''
    curses.echo()
    curses.nocbreak()
    curses.endwin()
    return


main()

