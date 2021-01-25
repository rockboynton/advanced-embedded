# HW 1 - RTOS Webinar

Rock Boynton

EE 4930/011

01/22/2020

## Summary

If you are writing software with any complexity on a embedded systems, Real Time Operating Systems (RTOS) provide many benefits over bare metal code including multithreading, concurrency, communications among different tasks, resource management, lower power consumption, and ability to assign priority to different tasks, beyond what just a scheduler can provide. There are many popular choices of RTOS for a project, like FreeRTOS and ThreadX, that can be used in a project instead of reinventing the wheel writing one yourself that you would need to maintain. Instead, you can view and edit the source code for these tried and tested solutions, adding whatever necessary to port to the processor and platform being used.

## RTOS Tasks

One significant thing I learned was about the how the tasks work in an RTOS. A task each acts as its own sort of mini bare-metal program, with task routine containing an infinite while loop similar to how a bare metal program would. Each task also contains its own stack for saving internal state of machine registers, a priority associated with it, and an ID for communicating with other tasks. Tasks can be scheduled to run fairly with a small amount of time allocated for each task, or they can have higher priority tasks always be run first only allowing lower priority tasks to run if nothing else is. Tasks can access shared resources such as memory. Synchronization primitives are provided by the RTOS like mutex and semaphores.
