#!/bin/bash

cmd_index=1
paused=0

cmd1='cansend can0 18EF8000#03FBFBFBFBFBFFFF; cansend can0 18EFC800#03FBFBFBFBFBFFFF; cansend can0 18EFF600#03FBFBFBFBFBFFFF'
cmd2='cansend can0 18EF8000#00FBFBFBFBFBFFFF; cansend can0 18EFC800#00FBFBFBFBFBFFFF; cansend can0 18EFF600#00FBFBFBFBFBFFFF'
cmd3='cansend can0 18EF8000#01FBFBFBFBFBFFFF; cansend can0 18EFC800#03FBFBFBFBFBFFFF; cansend can0 18EFF600#00FBFBFBFBFBFFFF'
cmd4='cansend can0 18EF8000#02FBFBFBFBFBFFFF; cansend can0 18EFC800#03FBFBFBFBFBFFFF; cansend can0 18EFF600#00FBFBFBFBFBFFFF'
cmd5='cansend can0 18EF8000#6400FBFBFBFBFFFF' #position in mm
#times 10 convert to hex, 05DC = 1500, 0320 = 80mm
#dont go past 0546 is 135mm to not hit hard stop
#Hex order is reversed, 0320 is 2003 in code
cmd6='cansend can0 18EFC800#01FBFBFBFBFBFFFF; cansend can0 18EF8000#03FBFBFBFBFBFFFF; cansend can0 18EFF600#00FBFBFBFBFBFFFF'
cmd7='cansend can0 18EFC800#02FBFBFBFBFBFFFF; cansend can0 18EF8000#03FBFBFBFBFBFFFF; cansend can0 18EFF600#00FBFBFBFBFBFFFF'
cmd8='cansend can0 18EFF600#01FBFBFBFBFBFFFF; cansend can0 18EFC800#03FBFBFBFBFBFFFF; cansend can0 18EF8000#03FBFBFBFBFBFFFF'
cmd9='cansend can0 18EFF600#02FBFBFBFBFBFFFF; cansend can0 18EFC800#03FBFBFBFBFBFFFF; cansend can0 18EF8000#03FBFBFBFBFBFFFF'

#cmd8='cansend can0 18EFC800#01FBFBFBFBFBFFFF; cansend can0 18EF8000#01FBFBFBFBFBFFFF'
#cmd9='cansend can0 18EFC800#02FBFBFBFBFBFFFF; cansend can0 18EF8000#02FBFBFBFBFBFFFF'
#cmd0='cansend can0 18EFC800#01FBFBFBFBFBFFFF; cansend can0 18EF8000#02FBFBFBFBFBFFFF'

#C8 is wrist 80 is shoulder dump is in Jesse notes app

while true; do
read -rsn1 -t 0.2 key

case "$key" in
1) cmd_index=1 ;;
2) cmd_index=2 ;;
3) cmd_index=3 ;;
4) cmd_index=4 ;;
5) cmd_index=5 ;;
6) cmd_index=6 ;;
7) cmd_index=7 ;;
8) cmd_index=8 ;;
9) cmd_index=9 ;;
q) break ;;
esac


case "$cmd_index" in
1) eval "$cmd1" ;;
2) eval "$cmd2" ;;
3) eval "$cmd3" ;;
4) eval "$cmd4" ;;
5) eval "$cmd5" ;;
6) eval "$cmd6" ;;
7) eval "$cmd7" ;;
8) eval "$cmd8" ;;
9) eval "$cmd9" ;;
esac

done
