# buzzer-code
Code for microcontroller-based quizbowl buzzer system,
requires Audio.h and malloc.h libraries as dependencies

This program operates a team "buzzer system" for up to eight quizbowl players. Quizbowl is an academic competition played between teams of students in which players compete to answer questions about literature, history, science, and more. The buzzer system recognizes whichever player "buzzes in" first and thus has the right to answer the question. Each player in this setup has a handheld controller that lights up and makes a noise when pressed. The noises are loaded into the microcontroller from an SD card beforehand.

Features:
- lights dim after the standard time limit of five seconds to answer a question run out
- keeps track of the second player to buzz in and automatically recognizes them when the system "clears", so long as they are still holding the button down.
- plays custom sound for each team, which were created from scratch using a synthesizer and a DAW
- debounce handling to mitigate the effects of mechanical signal noise
- detaches ISRs after buzzes to improve device performance
