#!/bin/bash
#store the currently-active window
export old_window=$(xdotool getactivewindow)
#get all windows that match the name
export my_window=($(xdotool search --name RobustnessStudyVideoStream | sed 's/:.*//'))
#get the last matching window
export my_window=${my_window[${#my_window[@]} - 1]}
#get the mouse position
export stuff=$(xdotool getmouselocation --shell)
#put it into $X and $Y
eval $stuff
#activate the desired window
xdotool windowactivate $my_window
#move the mouse over it
xdotool mousemove --window $my_window 20 20
#right click
xdotool click --window $my_window 3
#move the mouse back to its original position
xdotool mousemove $X $Y
#reactivate the originally active window
xdotool windowactivate $old_window

