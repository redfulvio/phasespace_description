# phasespace_description


phasespace_description is a code to reproduce "primitives" acquired using the [PhaseSpace Motion Capture System](http://phasespace.com/).

This package works on [Ubuntu 14.04](http://www.ubuntu.com/download/desktop) and needs the [pisa-iit-soft-hand](https://github.com/manuelbonilla/pisa-iit-soft-hand) package.


## Use

The commands to launch the programm is:

`roslaunch phasespace_description glove.launch`

then you have to publish which primitive you want to reproduce. Here there are three possibilities: "little" (lateral contact), "index" (lateral contact), "middle" (frontal contact). Use the following command:

`rostopic pub /which_finger std_msgs/String "data: 'x'" `

and substitute "x" with one of the three choices.
