# OwnTech Power API

This is the OwnTech Power API Core repository.

The Power API is designed to be used with VS Code and PlatformIO.
[Installing VS Code with PlatformIO](https://platformio.org/install/ide?install=vscode).

For information about Power API, check out its [Documentation](https://docs.owntech.org/#/renders/API/home).


## Downloading OwnTech Power API Core

You fisrt need to download the Power API Core repository using the following command:

`git clone https://github.com/owntech-foundation/Core.git owntech_power_api`

Then, open VS Code and, if not already done, install the PlatformIO plugin.

Finally, open the newly cloned folder `owntech_power_api` using menu `File > Open Folder...`


## Working with OwnTech Power API

While the project contains many folders and files, all your code goes to the `src` folder.
In the this folder, the file `main.cpp` is the entry point of the application.
Aditionally, some configuration can be done in the `platformio.ini` file.

This branch contains a ThingSet-controlled power test bench with a decimated
1,024-by-8 software scope. ThingSet control uses the board's `if02` USB-CDC
shell while frozen scope captures are downloaded over the separate `if00`
data interface. Its object tree, dual-serial protocol, safe examples, and
Python/MATLAB APIs are documented in [`src/README.md`](src/README.md).

Other folders and files are used to configure the underlying Zephyr OS and PlatformIO, and are hidden by default.


### Accessing OwnTech source code in VS Code (for advanced developers)

The full hierarchy of the project is as follows:

```
owntech_power_api
└─ owntech
|  └─ boards
|  └─ scripts
|  └─>pio_extra.ini
└─ src
|  └─>main.cpp
|  └─>user_data_objects.h
|  └─>user_data_objects.cpp
|  └─>thingset_callbacks.cpp
|  └─>scope_capture.h / scope_capture.cpp
|  └─>scope_data_port.h / scope_data_port.cpp
|  └─ tools
|     └─>thingset_tools.py / ThingSetTools.m
|     └─>power_test_bench.py / PowerTestBench.m
|     └─>scope_serial.py / ScopeSerial.m
└─ zephyr
|  └─ boards
|  └─ dts
|  └─ modules
|  └─>CMakeLists.txt
|  └─>prj.conf
└─>LICENSE
└─>platformio.ini
└─>README.md
```

The `owntech` folder contains scripts and board description for PlatformIO, while the `zephyr` folder contains board decription and OwnTech's Zephyr modules.
By default, these folders (as well as VS Code and PlatformIO folders `.vscode` and `.pio`) are hidden when opening the project in VS Code.

If you need to access these in VS Code, open the project using your file explorer, then in the `.vscode` folder, rename file `settings.json`, e.g. to `settings.json.old`.

Advanced Zephyr configuration can be tweaked by editing `zephyr/prj.conf`.

The OwnTech API source code is located in `zephyr/modules`. If you need to tailor it to your needs, please checkout the [Zephyr documentation](https://docs.zephyrproject.org/3.4.0/).
