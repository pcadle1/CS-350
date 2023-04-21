# CS-350
Summarize the project and what problem it was solving.
  The project implemented functionality to control a thermostat by using timers/interrupts and implementing a task scheduler. The code controls an LED on     the microcontroller as a proof of concept, which would correspond to a heater. Two buttons are used, one of which increases 
  the set point of the thermostat by one degree, and the other that decreases by one degree when pressed. The heater(LED) will turn on when the heat is       below the set temperature, and the heater will turn off once the set temperature is reached. The current temp, set point, status of the heater, and         elapsed time are output via UART every 1 second. Buttons are checked every 200ms, temp is checked every 500ms, and the output is presented every 1s. The   tasks are implemented via a task scheduler.
  
What did you do particularly well?
  I think I was able to organize the code well and write generally clean/understandable code. I think that the inclusion/implementation of the state         machine managing turning the heater on/off at the specified timer interval was logical. I also think that the documentation I included was quite clear.
  
Where could you improve?
  This was my first time working with a task scheduler implementation, and I think there is room for improvement in the task scheduler for this project.     One specific change could be creating variables for each task period, such that it can be more easily manipulated in the future. For example, the            buttons were to be checked every 200ms, but if we ended up changing it at some point to 400ms, then it would be easier to change if they were defined
  as variables.
  
What tools and/or resources are you adding to your support network?
  One of the broader tools that I'll be using more is diagramming before working, although I may transition to pen/paper for my individual pre-development   work. I did find the use of the debugger in code composer studio particularly useful during this work, so I'll be looking to incorporate using that
  earlier and more liberally when debugging in the future.

What skills from this project will be particularly transferable to other projects and/or course work?
  This project particularly helped me appreciate the value in creating diagrams early and understanding the problem as completely as possible before
  beginning. The "capture/convert" methodology was certainly helpful in this project, and I'll be trying to apply it moving forward. Once most of the 
  functionality was sorted out, I found it much quicker to produce code that was functional, and much quicker to debug/refactor as well.

How did you make this project maintainable, readable, and adaptable?
  I tried to make this project maintainable, readable, and adaptable by following as many of the best practices as I could. Notably, making the code well 
  organized, not cluttered, clear naming conventions/variables, documented functions, and separating repeated code into appropriate functions.
