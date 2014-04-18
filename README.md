wxSimAVR
========

wxWidgets wrapper for simavr library

simavr is a software simulator for the AVR range of microcontrollers. wxWidgets is a helper toolkit for building event driven (GUI) applications. wxSimAVR provides a wxWidgets library, allowing simple integration of simavr in a wxWidgets application.

wxSimAVR extends the wxThread class. The simulator runs in its own detached thread, communicating with the parent thread via wxWidgets events. Direct control of wxSimAVR is provided by public functions which are made thread safe by use of mutexes. This may result in small delays between a request being made and it being actioned whilst the thread completes its current action. This should not be an issue as the main thread loop just performs one simavr run cycle on each iteration.

Initial development is performed on Ubuntu 13.10 linux using wxWidgets 2.8.12. It should be possibleto be port the library to other platforms supporting wxWidgets and simavr with minimal effort.

This library is fully dependent on simavr <https://gitorious.org/simavr> which must be installed separately. The simavr headers must be available in the include search path.

The source code is initially developed within the Code::Blocks IDE and a Code::Blocks project is included in the source code.
